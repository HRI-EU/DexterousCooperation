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

#include "ExamplePolygonPlanner.h"

#include <ExampleFactory.h>
#include <Rcs_cmdLine.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>

#include "IKComponent.h"
#include "DyadicPolygonObjectConstraint.h"
#include "EventGui.h"
#include "EcsHardwareComponents.h"
#include "PolyGraspDetector.h"
#include "PolyROSComponent.h"
#include "BoxStrategy5D.h"

#include <AStar.h>
#include <ExtrudedPolygonNode.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_geometry.h>
#include <ForceDragger.h>
#include <COSNode.h>

#if defined (USE_ROS)
#include <ROSSpinnerComponent.h>
#include <VirtualKinect2.h>
#endif


namespace Dc
{
void trajectoryStarted(bool started)
{
  RLOG(0, "trajectoryStarted()");
  // runLoop = started;
  RFATAL("FIXME");
}

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger : public Rcs::ForceDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim) : ForceDragger(sim)
  {
  }

  virtual void update()
  {
    double f[3];
    Vec3d_sub(f, _I_mouseTip, _I_anchor);
    Vec3d_constMulSelf(f, getForceScaling() * (_leftControlPressed ? 10.0 : 1.0));

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
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePolygonPlanner, "Dexterous Cooperation", "Polygon planner");

ExamplePolygonPlanner::ExamplePolygonPlanner(int argc, char** argv) :
  ExampleBase(argc, argv), entity()
{
  Rcs::CmdLineParser argP(argc, argv);

  withGui = false;
  noEventGui = false;
  noSpeedCheck = false;
  noJointCheck = false;
  noCollCheck = false;
  seqSim = false;
  seqViewer = false;
  zigzag = false;
  pause = false;
  sync = false;
  noLimits = false;
  simpleGraphics = false;
  valgrind = false;
  polyDebug = false;
  physics = false;
  eternalTest = false;
  skipTrajectoryCheck = false;

  updateGraph = NULL;
  postUpdateGraph = NULL;
  computeKinematics = NULL;
  computeTrajectory = NULL;
  setTaskCommand = NULL;
  setJointCommand = NULL;
  setRenderCommand = NULL;

  dt = 0.01;
  dPhi = 30.0;
  ttc = 8.0;
  speedUp = 1;
  polyDirIdx = 3;
  loopCount = 0;

  viewer = NULL;
}

ExamplePolygonPlanner::~ExamplePolygonPlanner()
{
  delete viewer;
  //viewer.release();
  // controller.release();
  // graphC.release();
  // ikc.release();
  // trajC.release();
  // pop.release();
  // rebaC.release();
  // taskGui.release();
  pc.release();
  Rcs_removeResourcePath(directory.c_str());
}

bool ExamplePolygonPlanner::initParameters()
{
  directory = "config/xml/DexterousCooperation/PolygonTurning";
  cfgFile = "cPolyROS.xml";
  physicsEngine = "Bullet";
  physicsCfg = "config/physics/physics.xml";
  polyBaseDir = "config/xml/DexterousCooperation/PolygonTurning/polygons";
  dt = 0.01;
  dPhi = 30.0;
  ttc = 8.0;
  speedUp = 1;
  polyDirIdx = 3;
  loopCount = 0;
  polyDir = polyBaseDir + "/recordings" + std::to_string(polyDirIdx);

  return true;
}

bool ExamplePolygonPlanner::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-physicsEngine", &physicsEngine,
                      "Physics engine (default is \"%s\")", physicsEngine.c_str());
  parser->getArgument("-physics_config", &physicsCfg, "Configuration file name"
                      " for physics (default is %s)", physicsCfg.c_str());
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-ttc", &ttc, "Time between contacts (default is %f)", ttc);
  parser->getArgument("-dPhi", &dPhi, "Angular discretization in degrees (default"
                      " is %f)", dPhi);
  parser->getArgument("-speedUp", &speedUp, "Speed-up factor (default is %d)",
                      speedUp);
  parser->getArgument("-gui", &withGui, "Task gui (only visualization)");
  parser->getArgument("-noEventGui", &noEventGui, "Don't launch EventGui");
  parser->getArgument("-nospeed", &noSpeedCheck, "No speed limit checks");
  parser->getArgument("-nojl", &noJointCheck, "Disable joint limit checks");
  parser->getArgument("-nocoll", &noCollCheck, "Disable collision checks");
  parser->getArgument("-sequentialPhysics", &seqSim, "Physics simulation step"
                      "in updateGraph()");
  parser->getArgument("-sequentialViewer", &seqViewer, "Viewer frame call "
                      "in \"Render\" event");
  parser->getArgument("-zigzag", &zigzag, "Zig-zag trajectory");
  parser->getArgument("-pause", &pause, "Pause after each process() call");
  parser->getArgument("-sync", &sync, "Run as sequential as possible");
  parser->getArgument("-noLimits", &noLimits, "Ignore joint, speed and "
                      "collision limits");
  parser->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without "
                      "shadows and anti-aliasing");
  parser->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  parser->getArgument("-polyDebug", &polyDebug,
                      "Read polygons from file, not from ROS");
  parser->getArgument("-physics", &physics, "Run simulator");
  parser->getArgument("-eternalTest", &eternalTest, "Run eternal test");
  parser->getArgument("-skipTrajectoryCheck", &skipTrajectoryCheck,
                      "Perform plan without checking trajectory");

#if defined (USE_ROS)
  bool withROS = argP.hasArgument("-ros", "Connect to ROS");
  bool withVirtualKinect = argP.hasArgument("-kinect", "Enable virtual Kinect2");
#else
  polyDebug = true;
#endif

  dPhi *= M_PI / 180.0;

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

  if (parser->hasArgument("-h"))
  {
    entity.print(std::cout);
  }

  return true;
}

bool ExamplePolygonPlanner::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());
  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

  controller = std::make_unique<Rcs::ControllerBase>(cfgFile);

  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  setRenderCommand = entity.registerEvent<>("Render");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &ExamplePolygonPlanner::stop, this);

  graphC = std::make_unique<GraphComponent>(&entity, controller->getGraph());
  trajC = std::make_unique<TrajectoryComponent>(&entity, controller.get(), !zigzag, 1.0,
                                                !skipTrajectoryCheck);
  ikc = std::make_unique<IKComponent>(&entity, controller.get());
  ikc->setSpeedLimitCheck(!noSpeedCheck);
  ikc->setJointLimitCheck(!noJointCheck);
  ikc->setCollisionCheck(!noCollCheck);
  pop = std::make_unique<PolygonObjectPlanner>(&entity, controller.get(), dPhi);
  rebaC = std::make_unique<RebaComponent>(&entity, "cLinda.xml");

  return true;
}

bool ExamplePolygonPlanner::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

  // viewer = std::make_unique<Dc::GraphicsWindow>(&entity, false, seqViewer,
  //                                               simpleGraphics);
  viewer = new Dc::GraphicsWindow(&entity, false, seqViewer, simpleGraphics);

  // Display current polygon search path in graphics window
  entity.publish("SetTextLine", polyDir, 2);



  viewer->setKeyCallback('n', [this](char k)
  {
    entity.publish("RemoveNode", std::string(), std::string("Box_real"));

    double extrudeDir[3] = { 0.0, 0.0, -0.75 };
    const int nv = 100;
    const int nWaves = 6;
    double r_min = 0.1, r_max = 0.3;
    MatNd* polyArr = MatNd_create(nv, 2);
    double(*poly)[2] = (double(*)[2])polyArr->ele;
    Math_createRandomPolygon2D(poly, nv, nWaves, r_min, r_max);
    //Math_boxifyPolygon2D(poly, nv, 0.2);
    osg::ref_ptr<Rcs::NodeBase> node = new Rcs::ExtrudedPolygonNode(polyArr, extrudeDir);
    node->setName("Box_real");

    const RcsBody* box_v = RcsGraph_getBodyByName(ikc->getGraph(), "Box_v");
    RCHECK(box_v);
    HTr shapeTrf;
    HTr_copy(&shapeTrf, &box_v->A_BI);
    shapeTrf.org[2] += 0.1;
    node->setTransformation(&shapeTrf);


    entity.publish<osg::ref_ptr<osg::Node>>("AddNode", node);
    MatNd_destroy(polyArr);

  }, "Replace random polygon for depth rendering");

  viewer->setKeyCallback('d', [this](char k)
  {
    entity.publish("ResetErrorFlag");
  }, "Reseting IME failure flag");

  viewer->setKeyCallback('m', [this](char k)
  {
    entity.publish("MoveToInit", ttc);
  }, "Move to init state");

  viewer->setKeyCallback('h', [this](char k)
  {
    entity.publish("DriveHome", 10.0);
  }, "Drive home");

  viewer->setKeyCallback('a', [this](char k)
  {
    entity.publish("PlanToAngleAndExecute", M_PI_2, ttc);
  }, "Flip +90 degrees");

  viewer->setKeyCallback('b', [this](char k)
  {
    entity.publish("PlanToAngleAndExecute", M_PI, ttc);
  }, "Flip +180 degrees");

  viewer->setKeyCallback('o', [this](char k)
  {
    entity.publish("PlanToAngleAndExecute", 0.0, ttc);
  }, "Flip to 0 degrees");

  viewer->setKeyCallback('A', [this](char k)
  {
    entity.publish("PlanToAngleAndExecute", -M_PI_2, ttc);
  }, "Flip -90 degrees");

  viewer->setKeyCallback('B', [this](char k)
  {
    entity.publish("PlanToAngleAndExecute", -M_PI, ttc);
  }, "Flip -180 degrees");

  viewer->setKeyCallback('p', [this](char k)
  {
    std::string fn;
    if (polyDirIdx < 7)
    {
      static int count = 7;
      fn = polyDir + "/poly_" + std::to_string(count) + ".dat";
      count++;
      if (count > 9)
      {
        count = 0;
      }
    }
    else
    {
      static std::vector<std::string> polyFiles = Rcs::File_getFilenamesInDirectory(polyDir, false, ".dat");
      if (polyFiles.empty())
      {
        RLOG_CPP(0, "No polygon files found in directory " << polyDir);
        return;
      }

      static size_t count = 0;
      fn = polyDir + std::string("/") + polyFiles[count];
      count++;
      if (count >= polyFiles.size())
      {
        count = 0;
      }
    }

    entity.publish<std::string>("ReloadPolygon", fn);
    entity.publish("SetTextLine", fn, 2);
  }, "Load polygon from file");


  viewer->setKeyCallback('W', [this](char k)
  {
    polyDirIdx++;
    if (polyDirIdx > 7)
    {
      polyDirIdx = 2;
    }
    if (polyDirIdx < 7)
    {
      polyDir = polyBaseDir + "/recordings" + std::to_string(polyDirIdx) + "/";
    }
    else
    {
      polyDir = polyBaseDir + "/furniture/";
    }
    RLOG_CPP(0, "Changing to directory " << polyDir);
    entity.publish("SetTextLine", polyDir, 2);
  }, "Change polygon directory");

  viewer->setKeyCallback('l', [this](char k)
  {
    noLimits  = !noLimits;
    noSpeedCheck = noLimits;
    noJointCheck = noLimits;
    noCollCheck = noLimits;

    entity.publish("SetTrajectoryCheck", !noLimits);
    RLOG(0, "%s limits", noLimits ? "Disabling" : "Enabling");
  }, "Toggling limits");

  viewer->setKeyCallback('e', [this](char k)
  {
    RLOG(0, "Launch event gui");
    new EventGui(&entity);
  }, "Launch event gui");

  return true;
}

bool ExamplePolygonPlanner::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  if (withGui == true)
  {
    taskGui = std::make_unique<Dc::TaskGuiComponent>(&entity, controller.get());
    taskGui->setPassive(true);
  }

  return true;
}

/*******************************************************************************
 * bin/TestPolygonPlanner -m 1 -polyDebug -ime -ief -lbr1 -lbr2 -sdh1 -sdh2
 ******************************************************************************/
void ExamplePolygonPlanner::run()
{





  //Dc::ComponentFactory::print();
  std::vector<Dc::ComponentBase*> hwc = getHardwareComponents(entity, controller->getGraph());
  if (hwc.empty())
  {
    //physics = true;
  }

  if (physics)
  {
    // Make simulator use the position control interface
    RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    pc = std::make_unique<PhysicsComponent>(&entity, controller->getGraph(),
                                            physicsEngine.c_str(),
                                            physicsCfg.c_str(), !seqSim);
    if (pc->getPhysicsSimulation() && viewer)
    {
      viewer->add(new NamedBodyForceDragger(pc->getPhysicsSimulation()));
    }
    else
    {
      graphC->setEnableRender(false);
    }

  }

  Dc::PolyROSComponent poly(&entity, "/dexco/polygon", polyDebug);

#if defined (USE_ROS)
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









  if (noEventGui == false)
  {
    Dc::EventGui::create(&entity);
  }

  entity.initialize(graphC->getGraph());

  if (valgrind)
  {
    // entity.publish("PlanToAngleAndExecute", M_PI, 1.0);
    // entity.subscribe("TrajectoryMoving", &trajectoryStarted);
  }

  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));
  entity.publish("SetDebugRendering", false);



#if defined (USE_ROS)
  if (withVirtualKinect)
  {
    poly.setCameraBodyName("kinect2_dexco");
    entity.publish("SetViewFromKinect");
  }
#endif


  if (eternalTest)
  {
    pop->startEndlessTest();
  }

  while (runLoop)
  {
    step();
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

}

void ExamplePolygonPlanner::step()
{
  double dtProcess = Timer_getSystemTime();
  updateGraph->call(graphC->getGraph());
  computeKinematics->call(graphC->getGraph());
  computeTrajectory->call(ikc->getGraph());
  setTaskCommand->call(trajC->getActivationPtr(), trajC->getTaskCommandPtr());
  setJointCommand->call(ikc->getJointCommandPtr());
  setRenderCommand->call();
  postUpdateGraph->call(ikc->getGraph(), graphC->getGraph());
  entity.process();
  entity.stepTime();
  dtProcess = Timer_getSystemTime() - dtProcess;

  loopCount++;
  if ((loopCount % speedUp == 0) && (!valgrind))
  {
    Timer_waitDT(entity.getDt() - dtProcess);
  }

  if (valgrind)
  {
    RLOG_CPP(0, "Step " << loopCount);
    if (loopCount > 2)
    {
      runLoop = false;
    }
  }

}

std::string ExamplePolygonPlanner::help()
{
  std::string helpMsg = ExampleBase::help();
  helpMsg += entity.printToString();
  helpMsg += printHardwareComponentsToString();

  std::string usage = "Press \n"
                      "   W to select polygon directory\n"
                      "   p to load polygon\n"
                      "   m to move to initial pose\n"
                      "   a for rotating counter-clock-wise 90 deg\n"
                      "   A for rotating counter-clock-wise 180 deg\n"
                      "   b for rotating clock-wise 90 deg\n"
                      "   B for rotating clock-wise 180 deg\n";

  helpMsg += usage;

  return helpMsg;
}

}   // namespace
