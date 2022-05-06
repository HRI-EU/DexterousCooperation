/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "EntityBase.h"
#include "GraphComponent.h"
#include "GraphicsWindow.h"
#include "TaskGuiComponent.h"
#include "PhysicsComponent.h"
#include "IKComponent.h"
#include "DyadicPolygonObjectConstraint.h"
#include "PolygonObjectPlanner.h"
#include "EventGui.h"
#include "EcsHardwareComponents.h"
#include "TrajectoryComponent.h"
#include "PolyGraspDetector.h"
#include "PolyROSComponent.h"
#include "RebaComponent.h"

#include <ExtrudedPolygonNode.h>
#include <ControllerBase.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_geometry.h>
#include <Rcs_guiFactory.h>
#include <ForceDragger.h>
#include <COSNode.h>
#include <SegFaultHandler.h>

#if defined (USE_DC_ROS)
#include <ROSSpinnerComponent.h>
#include <VirtualKinect2.h>
#endif

#include <csignal>



RCS_INSTALL_ERRORHANDLERS


bool runLoop = true;



/*******************************************************************************
 *
 ******************************************************************************/
void sigQuit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

void quit()
{
  RLOG(0, "Quit::quit()");
  runLoop = false;
}

void trajectoryStarted(bool started)
{
  RLOG(0, "trajectoryStarted()");
  runLoop = started;
}

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger: public Rcs::ForceDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim) : ForceDragger(sim)
  {
  }

  virtual void update()
  {
    double f[3];
    Vec3d_sub(f, _I_mouseTip, _I_anchor);
    Vec3d_constMulSelf(f, getForceScaling()*(_leftControlPressed ? 10.0 : 1.0));

    RcsBody* simBdy = NULL;
    if (_draggedBody)
    {
      simBdy = RcsGraph_getBodyByName(physics->getGraph(), _draggedBody->name);
      NLOG(0, "Graph addy: 0x%x", physics->getGraph());
      NLOG(0, "Dragging %s to: [%f, %f, %f]", simBdy->name, f[0], f[1], f[2]);
    }
    physics->applyForce(simBdy, f, _k_anchor);

  }
};


/*******************************************************************************
 * bin/TestPolygonPlanner -m 3 -ime_odometry -ief -lbr1 -lbr2 -sdh1 -sdh2
 ******************************************************************************/
static void testIK(const char* cfgFile)
{
  Rcs::CmdLineParser argP;
  double dt = 0.01;
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  unsigned int speedUp = 1, loopCount = 0;
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-speedUp", &speedUp, "Speed-up factor (default is %d)",
                   speedUp);
  bool withGui = argP.hasArgument("-gui", "Task gui (only visualization)");
  bool noEventGui = argP.hasArgument("-noEventGui", "Don't launch EventGui");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step"
                                 "in updateGraph()");
  bool seqViewer = argP.hasArgument("-sequentialViewer", "Viewer frame call "
                                    "in \"Render\" event");
  bool zigzag = argP.hasArgument("-zigzag", "Zig-zag trajectory");
  bool pause = argP.hasArgument("-pause", "Pause after each process() call");
  bool sync = argP.hasArgument("-sync", "Run as sequential as possible");
  bool noLimits = argP.hasArgument("-noLimits", "Ignore joint, speed and "
                                   "collision limits");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  bool physics = argP.hasArgument("-physics", "Run simulator");
  bool skipTrajectoryCheck = argP.hasArgument("-skipTrajectoryCheck",
                                              "Perform plan without checking trajectory");

  if (noLimits)
  {
    noSpeedCheck = true;
    noJointCheck = true;
    noCollCheck = true;
  }

  if (sync)
  {
    seqSim = true;
    seqViewer = true;
  }

  if (valgrind)
  {
    noEventGui = true;
  }

  Rcs::EntityBase entity;
  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

  Rcs::ControllerBase* controller = new Rcs::ControllerBase(cfgFile);
  RcsGraph* graph = controller->getGraph();

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  std::shared_ptr<Rcs::GraphicsWindow> viewer;

  if (!valgrind)
  {
    viewer = std::make_shared<Rcs::GraphicsWindow>(&entity, false, seqViewer,
                                                   simpleGraphics);
  }

  Rcs::GraphComponent graphC(&entity, graph);
  Rcs::IKComponent ikc(&entity, controller);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);
  Rcs::TaskGuiComponent gui(&entity, controller);

  Rcs::ComponentFactory::print();
  std::vector<Rcs::ComponentBase*> hwc = getHardwareComponents(entity, graph);
  if (hwc.empty())
  {
    physics = true;
  }

  if (physics)
  {
    // Make simulator use the position control interface
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    Rcs::PhysicsComponent* pc = new Rcs::PhysicsComponent(&entity, graph,
                                                          physicsEngine,
                                                          physicsCfg, !seqSim);
    if (pc->getPhysicsSimulation() && viewer)
    {
      viewer->add(new NamedBodyForceDragger(pc->getPhysicsSimulation()));
    }
    else
    {
      graphC.setEnableRender(false);
    }

    hwc.push_back(pc);
  }

  delete controller;

  if (viewer)
  {
    viewer->setKeyCallback('d', [&entity](char k)
    {
      entity.publish("ResetErrorFlag");
    }, "Reseting IME failure flag");

  }   // if (viewer)


  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    return;
  }



  if (noEventGui == false)
  {
    Rcs::EventGui::create(&entity);
  }

  entity.initialize(graphC.getGraph());

  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"),
                 std::string("unsetGhostMode"));
  entity.publish("SetDebugRendering", false);



  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());
    computeKinematics->call(graphC.getGraph());
    setTaskCommand->call(gui.getActivationPtr(), gui.getTaskCommandPtr());
    setJointCommand->call(ikc.getJointCommandPtr());
    setRenderCommand->call();
    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());
    entity.process();
    entity.stepTime();
    dtProcess = Timer_getSystemTime() - dtProcess;

    loopCount++;
    if (loopCount%speedUp==0)
    {
      Timer_waitDT(entity.getDt() - dtProcess);
    }
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

}

/*******************************************************************************
 * bin/TestPolygonPlanner -m 1 -polyDebug -ime -ief -lbr1 -lbr2 -sdh1 -sdh2
 ******************************************************************************/
static void testROS(const char* cfgFile)
{
  Rcs::CmdLineParser argP;
  double dt = 0.01, dPhi = 30.0, ttc = 8.0;
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  std::string polyDir = "config/xml/DexterousCooperation/PolygonTurning/polygons/recordings";
  int polyDirIdx = 3;
  unsigned int speedUp = 1, loopCount = 0;
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-ttc", &ttc, "Time between contacts (default is %f)", ttc);
  argP.getArgument("-dPhi", &dPhi, "Angular discretization in degrees (default"
                   " is %f)", dPhi);
  argP.getArgument("-speedUp", &speedUp, "Speed-up factor (default is %d)",
                   speedUp);
  bool withGui = argP.hasArgument("-gui", "Task gui (only visualization)");
  bool noEventGui = argP.hasArgument("-noEventGui", "Don't launch EventGui");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step"
                                 "in updateGraph()");
  bool seqViewer = argP.hasArgument("-sequentialViewer", "Viewer frame call "
                                    "in \"Render\" event");
  bool zigzag = argP.hasArgument("-zigzag", "Zig-zag trajectory");
  bool pause = argP.hasArgument("-pause", "Pause after each process() call");
  bool sync = argP.hasArgument("-sync", "Run as sequential as possible");
  bool noLimits = argP.hasArgument("-noLimits", "Ignore joint, speed and "
                                   "collision limits");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  bool polyDebug = argP.hasArgument("-polyDebug",
                                    "Read polygons from file, not from ROS");
  bool physics = argP.hasArgument("-physics", "Run simulator");
  bool eternalTest = argP.hasArgument("-eternalTest", "Run eternal test");
  bool skipTrajectoryCheck = argP.hasArgument("-skipTrajectoryCheck",
                                              "Perform plan without checking trajectory");
#if defined (USE_DC_ROS)
  bool withROS = argP.hasArgument("-ros", "Connect to ROS");
  bool withVirtualKinect = argP.hasArgument("-kinect", "Enable virtual Kinect2");
#else
  polyDebug = true;
#endif

  dPhi *= M_PI/180.0;

  if (noLimits)
  {
    noSpeedCheck = true;
    noJointCheck = true;
    noCollCheck = true;
  }

  if (sync)
  {
    seqSim = true;
    seqViewer = true;
  }

  if (valgrind)
  {
    noEventGui = true;
  }

  Rcs::EntityBase entity;
  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

  Rcs::ControllerBase* controller = new Rcs::ControllerBase(cfgFile);
  RcsGraph* graph = controller->getGraph();

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  auto setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  std::shared_ptr<Rcs::GraphicsWindow> viewer;

  if (!valgrind)
  {
    viewer = std::make_shared<Rcs::GraphicsWindow>(&entity, false, seqViewer,
                                                   simpleGraphics);
  }

  Rcs::GraphComponent graphC(&entity, graph);
  Rcs::TrajectoryComponent trajC(&entity, controller, !zigzag, 1.0,
                                 !skipTrajectoryCheck);
  Rcs::IKComponent ikc(&entity, controller);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);
  Rcs::PolygonObjectPlanner pop(&entity, controller, dPhi);
  Rcs::RebaComponent rebaC(&entity, "cLinda.xml");

  std::shared_ptr<Rcs::TaskGuiComponent> taskGui;
  if (withGui == true)
  {
    taskGui = std::make_shared<Rcs::TaskGuiComponent>(&entity, controller);
    taskGui->setPassive(true);
  }

  Rcs::ComponentFactory::print();
  std::vector<Rcs::ComponentBase*> hwc = getHardwareComponents(entity, graph);
  if (hwc.empty())
  {
    physics = true;
  }

  if (physics)
  {
    // Make simulator use the position control interface
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    Rcs::PhysicsComponent* pc = new Rcs::PhysicsComponent(&entity, graph,
                                                          physicsEngine,
                                                          physicsCfg, !seqSim);
    if (pc->getPhysicsSimulation() && viewer)
    {
      viewer->add(new NamedBodyForceDragger(pc->getPhysicsSimulation()));
    }
    else
    {
      graphC.setEnableRender(false);
    }

    hwc.push_back(pc);
  }

  Rcs::PolyROSComponent poly(&entity, "/dexco/polygon", polyDebug);

#if defined (USE_DC_ROS)
  if (withROS)
  {
    hwc.push_back(new Rcs::ROSSpinnerComponent(&entity, 1));
  }

  if (withVirtualKinect)
  {
    Rcs::VirtualKinect2* vk2 = new Rcs::VirtualKinect2(&entity);
    Rcs::GraphNode* gn = new Rcs::GraphNode(ikc.getGraph());//graphC.getGraph() for true robot pose
    gn->toggleDepthModel();
    gn->displayCollisionModel(false);
    gn->displayGraphicsModel(false);
    gn->displayPhysicsModel(false);
    gn->displayReferenceFrames(false);
    vk2->addNode(gn);
    hwc.push_back(vk2);
  }
#endif

  delete controller;

  if (viewer)
  {
    viewer->setKeyCallback('n', [&entity, &ikc](char k)
    {
      entity.publish("RemoveNode", std::string(), std::string("Box_real"));

      double extrudeDir[3] = {0.0, 0.0, -0.75};
      const int nv = 100;
      const int nWaves = 6;
      double r_min = 0.1, r_max = 0.3;
      MatNd* polyArr = MatNd_create(nv, 2);
      double (*poly)[2] = (double (*)[2])polyArr->ele;
      Math_createRandomPolygon2D(poly, nv, nWaves, r_min, r_max);
      //Math_boxifyPolygon2D(poly, nv, 0.2);
      osg::ref_ptr<Rcs::NodeBase> node = new Rcs::ExtrudedPolygonNode(polyArr, extrudeDir);
      node->setName("Box_real");

      const RcsBody* box_v = RcsGraph_getBodyByName(ikc.getGraph(), "Box_v");
      RCHECK(box_v);
      HTr shapeTrf;
      HTr_copy(&shapeTrf, &box_v->A_BI);
      shapeTrf.org[2] += 0.1;
      node->setTransformation(&shapeTrf);


      entity.publish<osg::ref_ptr<osg::Node>>("AddNode", node);
      MatNd_destroy(polyArr);

    }, "Replace random polygon for depth rendering");

    viewer->setKeyCallback('d', [&entity](char k)
    {
      entity.publish("ResetErrorFlag");
    }, "Reseting IME failure flag");

    viewer->setKeyCallback('m', [&entity, &ttc](char k)
    {
      entity.publish("MoveToInit", ttc);
    }, "Move to init state");

    viewer->setKeyCallback('h', [&entity](char k)
    {
      entity.publish("DriveHome", 10.0);
    }, "Drive home");

    viewer->setKeyCallback('a', [&entity, &ttc](char k)
    {
      entity.publish("PlanToAngleAndExecute", M_PI_2, ttc);
    }, "Flip +90 degrees");

    viewer->setKeyCallback('b', [&entity, &ttc](char k)
    {
      entity.publish("PlanToAngleAndExecute", M_PI, ttc);
    }, "Flip +180 degrees");

    viewer->setKeyCallback('o', [&entity, &ttc](char k)
    {
      entity.publish("PlanToAngleAndExecute", 0.0, ttc);
    }, "Flip to 0 degrees");

    viewer->setKeyCallback('A', [&entity, &ttc](char k)
    {
      entity.publish("PlanToAngleAndExecute", -M_PI_2, ttc);
    }, "Flip -90 degrees");

    viewer->setKeyCallback('B', [&entity, &ttc](char k)
    {
      entity.publish("PlanToAngleAndExecute", -M_PI, ttc);
    }, "Flip -180 degrees");

    viewer->setKeyCallback('p', [&entity, &polyDir, &polyDirIdx](char k)
    {
      static int count = 7;
      std::string dirName = polyDir + std::to_string(polyDirIdx);
      std::string fn = dirName + "/poly_" + std::to_string(count) + ".dat";
      entity.publish<std::string>("ReloadPolygon", fn);
      entity.publish("SetTextLine", fn, 2);
      count++;
      if (count>9)
      {
        count = 0;
      }
    }, "Load polygon from file");


    viewer->setKeyCallback('W', [&polyDir, &polyDirIdx](char k)
    {
      polyDirIdx++;
      if (polyDirIdx>6)
      {
        polyDirIdx = 2;
      }
      std::string dirName = polyDir + std::to_string(polyDirIdx);
      RLOG_CPP(0, "Changing to directory " << dirName);
    }, "Change polygon directory");

  }   // if (viewer)


  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    return;
  }



  if (noEventGui == false)
  {
    Rcs::EventGui::create(&entity);
  }

  entity.initialize(graphC.getGraph());

  if (valgrind)
  {
    entity.publish("PlanToAngleAndExecute", M_PI, 1.0);
    entity.subscribe("TrajectoryMoving", &trajectoryStarted);
  }

  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"),
                 std::string("unsetGhostMode"));
  entity.publish("SetDebugRendering", false);



#if defined (USE_DC_ROS)
  if (withVirtualKinect)
  {
    poly.setCameraBodyName("kinect2_dexco");
    entity.publish("SetViewFromKinect");
  }
#endif


  if (eternalTest)
  {
    pop.startEndlessTest();
  }

  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());
    computeKinematics->call(graphC.getGraph());
    computeTrajectory->call(ikc.getGraph());
    setTaskCommand->call(trajC.getActivationPtr(), trajC.getTaskCommandPtr());
    setJointCommand->call(ikc.getJointCommandPtr());
    setRenderCommand->call();
    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());
    entity.process();
    entity.stepTime();
    dtProcess = Timer_getSystemTime() - dtProcess;

    loopCount++;
    if (loopCount%speedUp==0)
    {
      Timer_waitDT(entity.getDt() - dtProcess);
    }
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

}

/*******************************************************************************
 * Create polygon from file
 ******************************************************************************/
static MatNd* createFromFile(std::string polyFileName)
{
  MatNd* polyArr = NULL;
  double scale = 1.0;

  if (polyFileName.empty() || polyFileName=="Box")
  {
    double radius = 0.18;
    double height = 0.63;
    polyArr = MatNd_create(4, 2);
    MatNd_set(polyArr, 0, 0, -radius);
    MatNd_set(polyArr, 0, 1,-0.5*height);
    MatNd_set(polyArr, 1, 0, radius);
    MatNd_set(polyArr, 1, 1,-0.5*height);
    MatNd_set(polyArr, 2, 0, radius);
    MatNd_set(polyArr, 2, 1, 0.5*height);
    MatNd_set(polyArr, 3, 0,-radius);
    MatNd_set(polyArr, 3, 1, 0.5*height);
    RCHECK(Math_checkPolygon2D((double(*)[2])polyArr->ele, polyArr->m));
  }
  else if (polyFileName=="Cylinder")
  {
    const size_t nVertices = 128;
    const double radius = 0.3;
    const double sliceAngle = 2.0*M_PI/nVertices;
    double phi0 = 0.0;

    polyArr = MatNd_create(nVertices, 2);

    if (nVertices % 2 == 0)
    {
      phi0 = -0.5*sliceAngle;
    }

    for (size_t i=0; i<nVertices; ++i)
    {
      double phiNormal = phi0 + i*sliceAngle - M_PI_2;
      double x = radius*cos(phiNormal+M_PI_2);
      double y = radius*sin(phiNormal+M_PI_2);
      MatNd_set(polyArr, i, 0, x);
      MatNd_set(polyArr, i, 1, y);
    }

  }
  else
  {
    polyArr = MatNd_createFromFile(polyFileName.c_str());
  }

  if (polyArr)
  {
    MatNd_constMulSelf(polyArr, scale);
  }


  return polyArr;
}

/*******************************************************************************
 * 2D polygon test
 * if we define dx=x2-x1 and dy=y2-y1, then the normals are
 * (-dy, dx) and (dy, -dx).
 ******************************************************************************/
void testPolyFromClass(int argc, char** argv)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("f", "Load next file");
  Rcs::KeyCatcherBase::registerKey("+",
                                   "Change visualization (for detail=2 only)");

  std::string polyFileName;
  int detail = 0;
  double resampleSize = 0.02;
  int halfFilter = 5;
  double maxLineFitError = 0.01;
  double handWidth = 0.25;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-f", &polyFileName, "Polygon file name");
  argP.getArgument("-detail", &detail, "Detail of visualization (0-2, "
                   "default: %d)", detail);
  argP.getArgument("-resampleSize", &resampleSize,
                   "Resampling distance (default: %f)", resampleSize);
  argP.getArgument("-halfFilter", &halfFilter,
                   "Half filter steps (default: %d)", halfFilter);
  argP.getArgument("-maxLineFitError", &maxLineFitError,
                   "Max. line fit error (default: %f)", maxLineFitError);
  argP.getArgument("-handWidth", &handWidth, "Hand width (default: %f)",
                   handWidth);

  if (argP.hasArgument("-h"))
  {
    return;
  }

  // Initialize GUI and OSG mutex
  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->setCameraTransform(0.03, -0.09, 1.70, -1.53, -0.61, 1.58);
  viewer->setTrackballCenter(0.0, 0.0, 0.0);
  Rcs::PolyGraspDetector graspDetector;

  //////////////////////////////////////////////////////////////////////////////
  // Create polygon rectangle, or from data file
  //////////////////////////////////////////////////////////////////////////////
  MatNd* polyArr = createFromFile(polyFileName);
  RLOG(1, "Created polygon file with %d vertices", polyArr ? polyArr->m : -1);
  graspDetector.setPolygon(polyArr, resampleSize, halfFilter, maxLineFitError,
                           handWidth);
  MatNd_destroy(polyArr);
  auto nodes = graspDetector.createVisualizations(0.05, detail);
  for (size_t i=0; i<nodes.size(); ++i)
  {
    viewer->add(nodes[i]);
  }

  Rcs::HUD* hud = new Rcs::HUD();
  viewer->add(hud);
  Rcs::KeyCatcher* kc = new Rcs::KeyCatcher();
  viewer->add(kc);
  viewer->add(new Rcs::COSNode(0.1));
  viewer->runInThread(&mtx);

  size_t displayIdx = 0;


  while (runLoop)
  {
    pthread_mutex_lock(&mtx);
    hud->setText(polyFileName);
    pthread_mutex_unlock(&mtx);

    if (kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc->getAndResetKey('f'))
    {
      int nextIdx = 0;

      if (!polyFileName.empty())
      {
        std::string sub = polyFileName.substr(polyFileName.length()-5, 1);
        nextIdx = atoi(sub.c_str())+1;
        RLOG(1, "Next index is %d %s", nextIdx, sub.c_str());
      }

      std::string newFile = "config/data/polygons/recordings1/poly_" +
                            std::to_string(nextIdx) + ".dat";

      polyArr = createFromFile(newFile);

      if (polyArr)
      {
        polyFileName = newFile;
        viewer->removeNodes();
        pthread_mutex_lock(&mtx);
        double tPoly = Timer_getSystemTime();
        graspDetector.setPolygon(polyArr, resampleSize, halfFilter,
                                 maxLineFitError, handWidth);
        tPoly = Timer_getSystemTime() - tPoly;
        pthread_mutex_unlock(&mtx);
        MatNd_destroy(polyArr);

        nodes = graspDetector.createVisualizations(0.05, detail);
        for (size_t i=0; i<nodes.size(); ++i)
        {
          viewer->add(nodes[i]);
        }

        RLOG(0, "Polygon file %s took %.3f msec",
             polyFileName.c_str(), 1.0e3*tPoly);
      }

    }
    else if (kc->getAndResetKey('+'))
    {
      if (detail==2)
      {
        displayIdx++;
        if (displayIdx > nodes.size()-1)
        {
          displayIdx = 0;
        }

        RLOG_CPP(0, "Displaying node " << displayIdx+1 << " from "
                 << nodes.size());

        for (size_t i=0; i<nodes.size(); ++i)
        {
          if (i==displayIdx)
          {
            nodes[i]->show();
          }
          else
          {
            nodes[i]->hide();
          }
        }
      }
    }

    Timer_waitDT(0.01);
  }

  delete viewer;
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  char xmlFileName[128] = "cPolyROS.xml";
  char directory[128] = "config/xml/DexterousCooperation/PolygonTurning";
  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  // Ctrl-C callback handler
  signal(SIGINT, sigQuit);

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  switch (mode)
  {
    case 0:
      printf("\tTestPolygonOutline:\n");
      printf("\tMode 1: Test with receiving polygons through ROS\n");
      printf("\tMode 2: Polygons from file with graphics window\n");
#if defined (USE_DC_ROS)
      printf("\tROS is enabled\n");
#else
      printf("\tROS is disabled\n");
#endif
      break;

    case 1:
      testROS(xmlFileName);
      break;

    case 2:
      testPolyFromClass(argc, argv);
      break;

    case 3:
      testIK("cInvKin.xml");
      break;

    default:
      RMSG("No mode %d", mode);
  }



  if (argP.hasArgument("-h"))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
  }


  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
