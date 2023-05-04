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

#include <EventSystem.h>
#include <EntityBase.h>
#include <GraphComponent.h>
#include <GraphicsWindow.h>
#include <TaskGuiComponent.h>
#include <JointGuiComponent.h>
#include <PhysicsComponent.h>
#include <IKComponent.h>
#include <EventGui.h>
#include <EcsHardwareComponents.h>
#include <BoxObjectChanger.h>
#include <ColliderComponent.h>
#include <TTSComponent.h>


#include <ActivationSet.h>
#include <PouringConstraint.h>

#if defined (USE_OPENCV_ARUCO)
#include <ArucoComponent.h>
#endif

#if defined (USE_ROS)
#include <ROSSpinnerComponent.h>
#endif

#include <TrajectoryComponent.h>
#include <JacoConstraint.h>
#include <ConstraintFactory.h>

#include <ControllerBase.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_graphParser.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_guiFactory.h>
#include <NamedBodyForceDragger.h>
#include <BulletDebugNode.h>
#include <SegFaultHandler.h>

RCS_INSTALL_ERRORHANDLERS

using namespace Rcs;

bool runLoop = true;


/*******************************************************************************
 *
 ******************************************************************************/
void quit()
{
  RLOG(0, "Quit::quit()");
  runLoop = false;
}

/*******************************************************************************
 * Joint-level control with Gui
 ******************************************************************************/
static void testJointGui(int argc, char** argv)
{
  Dc::EntityBase entity;

  CmdLineParser argP;
  double dt = 0.005, tmc=0.25;
  char xmlFileName[128] = "cSimpleBox.xml";
  char directory[128] = "config/xml/DexterousCooperation/BoxTurning";
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-tmc", &tmc, "Time constant applied to slider values filter "
                   "(default is %f)", tmc);
  bool withROS = argP.hasArgument("-ros", "Initialize ROS");
  bool startViewerWithStartEvent = argP.hasArgument("-h");
  Rcs_addResourcePath(directory);

  entity.setDt(dt);

  RcsGraph* graph = RcsGraph_create(xmlFileName);

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  Dc::GraphicsWindow viewer(&entity, startViewerWithStartEvent);
  Dc::GraphComponent graphC(&entity, graph);
  Dc::JointGuiComponent gui(&entity, graph, tmc);
  Dc::BoxObjectChanger objC(&entity);

  std::vector<Dc::ComponentBase*> hwc = Dc::getHardwareComponents(entity, graph);

#if defined (USE_ROS)
  //ros::init(argc, argv, "RcsROS", ros::init_options::NoSigintHandler);
  //RPAUSE();
  if (withROS)
  {
    Rcs::ROSSpinnerComponent spinner(&entity);
  }
  //RPAUSE();
#endif

  if (hwc.empty())
  {
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }
    hwc.push_back(new Dc::PhysicsComponent(&entity, graph, physicsEngine, physicsCfg, false));
  }

  viewer.setKeyCallback('m', [&gui, &graphC](char k)
  {
    RcsGraph* graph = graphC.getGraph();
    std::string mdlState;
    RMSG("Changing model state. Options are:\n");
    std::vector<std::string> ms = RcsGraph_getModelStateNames(graph);
    for (size_t i=0; i<ms.size(); ++i)
    {
      printf("\t%s\n", ms[i].c_str());
    }
    printf("\nEnter name of model state: ");
    std::cin >> mdlState;

    MatNd* q_goal = MatNd_clone(graph->q);
    bool ok = RcsGraph_getModelStateFromXML(q_goal, graph, mdlState.c_str(), 0);

    if (ok)
    {
      gui.setGoalPose(q_goal);
    }

    RMSG("%s changing model state to %s",
         ok ? "SUCCEEDED" : "FAILED", mdlState.c_str());
    MatNd_destroy(q_goal);

  }, "Move to model state");

  RcsGraph_destroy(graph);

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    for (size_t i=0; i<hwc.size(); ++i)
    {
      delete hwc[i];
    }
    return;
  }

  new Dc::EventGui(&entity);

  entity.initialize(graphC.getGraph());

  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());
    computeKinematics->call(graphC.getGraph());
    setJointCommand->call(gui.getJointCommandPtr());
    setRenderCommand->call();
    postUpdateGraph->call(NULL, graphC.getGraph());

    entity.process();
    dtProcess = Timer_getSystemTime() - dtProcess;

    char text[256];
    sprintf(text, "process() took %.3f msec", 1.0e3*dtProcess);
    entity.publish<std::string>("SetText", std::string(text));

    Timer_waitDT(entity.getDt());
    RPAUSE_DL(5);
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }
}

/*******************************************************************************
 * Task-level control with IK and Gui
 ******************************************************************************/
static void testTaskGui()
{
  CmdLineParser argP;
  double dt = 0.01;
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  char xmlFileName[128] = "cSimpleBox.xml";
  char directory[128] = "config/xml/DexterousCooperation/BoxTurning";
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  bool pause = argP.hasArgument("-pause", "Pause after each process() call");
  bool withGui = argP.hasArgument("-gui", "Launch event gui");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step"
                                 "in updateGraph()");
  bool seqViewer = argP.hasArgument("-sequentialViewer", "Viewer frame call "
                                    "in \"Render\" event");
  bool sync = argP.hasArgument("-sync", "Run as sequential as possible");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool noLimits = argP.hasArgument("-noLimits", "Ignore joint, speed and "
                                   "collision limits");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");

  Rcs_addResourcePath(directory);

  Dc::EntityBase entity;

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    return;
  }

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

  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

  ControllerBase* controller = new ControllerBase(xmlFileName);
  RcsGraph* graph = controller->getGraph();

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  Dc::GraphicsWindow viewer(&entity, false, seqViewer, simpleGraphics);
  Dc::GraphComponent graphC(&entity, graph);
  Dc::TaskGuiComponent gui(&entity, controller);
  Dc::IKComponent ikc(&entity, controller, Dc::IKComponent::IkSolverType::RMR);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);
  Dc::BoxObjectChanger objC(&entity);

  std::vector<std::string> b1, b2;
  b1.push_back("Box_real");
  //b2.push_back("sdh-base_L");
  //b2.push_back("sdh-base_R");
  b2.push_back("kinect2_dexco");

  Dc::ColliderComponent collider(&entity, b1, b2, false);
  std::vector<Dc::ComponentBase*> hwc = Dc::getHardwareComponents(entity, graph);
  Dc::PhysicsComponent* pc = NULL;

  if (hwc.empty())
  {
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    pc = new Dc::PhysicsComponent(&entity, graph, physicsEngine,
                                  physicsCfg, !seqSim);
    if (pc->getPhysicsSimulation())
    {
      osg::ref_ptr<NamedBodyForceDragger> dragger =
        new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
    }
    else
    {
      graphC.setEnableRender(false);
    }

    if (seqSim && seqViewer)
    {
#if defined (USE_BULLET)
      osg::ref_ptr<BulletDebugNode> btNd =
        new BulletDebugNode(pc->getPhysicsSimulation(), viewer.getFrameMtx());
      btNd->show();
      viewer.add(btNd.get());
#endif
    }

    hwc.push_back(pc);
  }

  delete controller;

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    return;
  }

  // Start threads (if any)
  if (withGui==true)
  {
    new Dc::EventGui(&entity);
  }

  entity.initialize(graphC.getGraph());

  if (pc && (!pc->getPhysicsSimulation()))
  {
    entity.publish("RenderCommand", std::string("IK"),
                   std::string("unsetGhostMode"));
  }




  while (runLoop)
  {
    double dtProcess0 = Timer_getSystemTime();

    // Read sensor values into graph
    updateGraph->call(graphC.getGraph());

    // Updates the TaskGui and filter the commands
    computeKinematics->call(graphC.getGraph());
    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());

    // Compute IK with the gui's inputs into the IKComponents internal q_des
    setTaskCommand->call(gui.getActivationPtr(), gui.getTaskCommandPtr());

    // Distribute the IKComponents internal command to all motor components
    setJointCommand->call(ikc.getJointCommandPtr());

    // This triggers graphics updates in some components
    setRenderCommand->call();

    double dtProcess1 = Timer_getSystemTime();

    entity.process();

    double dtProcess2 = Timer_getSystemTime();

    char text[256];
    sprintf(text, "Time: %.3f\nprocess() took %.3f + %.3f msec",
            entity.getTime(), 1.0e3*(dtProcess1-dtProcess0),
            1.0e3*(dtProcess2-dtProcess1));
    entity.publish<std::string>("SetText", std::string(text));

    entity.stepTime();
    Timer_waitDT(entity.getDt()-(dtProcess2-dtProcess0));
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

}

/*******************************************************************************
 * Task-level control with trajectory generation
 ******************************************************************************/
class JacoHandOpener : public Dc::ComponentBase
{
public:
  JacoHandOpener(Dc::EntityBase* parent) : Dc::ComponentBase(parent), t_blind(0.0), isOpen(false)
  {
    RLOG(0, "Creating JacoHandOpener");
    subscribe("JacoHandForce", &JacoHandOpener::onOpenHand);
  }


private:

  void onOpenHand(double force)
  {
    double t = Timer_getTime();

    // We reject any signal that comes within the next 3 secs
    if (t - t_blind < 3.0)
    {
      return;
    }

    t_blind = t;

    RLOG(0, "Force of %f detected", force);
    Rcs::JacoConstraint c;

    if (isOpen)
    {
      auto tSet = c.closeFingers(0.0, 2.0);
      getEntity()->publish("SetTrajectory", tSet);
    }
    else
    {
      auto tSet = c.openFingers(0.0, 2.0);
      getEntity()->publish("SetTrajectory", tSet);
    }

    isOpen = !isOpen;
  }

  double t_blind;
  bool isOpen;
};


static void testTrajectory()
{
  CmdLineParser argP;
  double dt = 0.01;
  unsigned int speedUp = 1, loopCount = 0;
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  char xmlFileName[128] = "cAction.xml";
  char directory[128] = "config/xml/Scitos";
  argP.getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)",
                   speedUp);
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  bool pause = argP.hasArgument("-pause", "Pause after each process() call");
  bool withGui = argP.hasArgument("-gui", "Launch event gui");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step"
                                 "in updateGraph()");
  bool seqViewer = argP.hasArgument("-sequentialViewer", "Viewer frame call "
                                    "in \"Render\" event");
  bool sync = argP.hasArgument("-sync", "Run as sequential as possible");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool noLimits = argP.hasArgument("-noLimits", "Ignore joint, speed and "
                                   "collision limits");
  bool zigzag = argP.hasArgument("-zigzag", "Zig-zag trajectory");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");

  Rcs_addResourcePath(directory);

  Dc::EntityBase entity;

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    RMSG("Usage:\nbin/ExampleDcComponents -m 3 -jaco");
    return;
  }

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

  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

  ControllerBase* controller = new ControllerBase(xmlFileName);
  RcsGraph* graph = controller->getGraph();

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  auto setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  JacoHandOpener handOpener(&entity);
  Dc::TrajectoryComponent trajC(&entity, controller, !zigzag, 1.0, false);
  Dc::GraphicsWindow viewer(&entity, false, seqViewer, simpleGraphics);
  Dc::GraphComponent graphC(&entity, graph);
  Dc::BoxObjectChanger objC(&entity);
  Dc::IKComponent ikc(&entity, controller, Dc::IKComponent::IkSolverType::RMR);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);
  Dc::TaskGuiComponent gui(&entity, controller);
  gui.setPassive(true);

  std::vector<Dc::ComponentBase*> hwc = Dc::getHardwareComponents(entity, graph);
  Dc::PhysicsComponent* pc = NULL;

  if (hwc.empty())
  {
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    pc = new Dc::PhysicsComponent(&entity, graph, physicsEngine,
                                  physicsCfg, !seqSim);

    if (pc->getPhysicsSimulation())
    {
      osg::ref_ptr<NamedBodyForceDragger> dragger =
        new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
    }
    else
    {
      graphC.setEnableRender(false);
    }

    if (seqSim && seqViewer)
    {
#if defined (USE_BULLET)
      osg::ref_ptr<BulletDebugNode> btNd =
        new BulletDebugNode(pc->getPhysicsSimulation(), viewer.getFrameMtx());
      btNd->show();
      viewer.add(btNd.get());
#endif
    }

    hwc.push_back(pc);
  }

  delete controller;

  viewer.setKeyCallback('a', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.home(0.0, 8.0);
    //entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
    entity.publish("SetTrajectory", tSet);
  }, "Move home");

  viewer.setKeyCallback('o', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.get(0.0, 8.0, 1);
    RLOG(0, "XML test %s", tSet->testXML() ? "SUCCEEDED" : "FAILED");
    entity.publish("SetTrajectory", tSet);
  }, "Get object 1");

  viewer.setKeyCallback('u', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.release(0.0, 8.0, 1);
    entity.publish("SetTrajectory", tSet);
  }, "Get object 1");

  viewer.setKeyCallback('z', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.put(0.0, 8.0, 1);
    entity.publish("SetTrajectory", tSet);
  }, "Put object 1");

  viewer.setKeyCallback('O', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.get(0.0, 8.0, 2);
    //entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
    //entity.process();
    entity.publish("SetTrajectory", tSet);
  }, "Get object 2");

  viewer.setKeyCallback('U', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.release(0.0, 8.0, 2);
    entity.publish("SetTrajectory", tSet);
  }, "Get object 1");

  viewer.setKeyCallback('Z', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.put(0.0, 8.0, 2);
    entity.publish("SetTrajectory", tSet);
  }, "Get object 2");

  viewer.setKeyCallback('y', [&entity, &graphC](char k)
  {
    Rcs::JacoConstraint c;
    auto tSet = c.pour(0.0, 8.0, 2);
    //entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
    entity.publish("SetTrajectory", tSet);
  }, "Pour object 2");

  viewer.setKeyCallback('l', [&entity, &graphC](char k)
  {
    entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
  }, "Init from state");

  viewer.setKeyCallback('L', [&entity, &graphC](char k)
  {
    RLOG_CPP(0, "Please enter constraint file name:");
    std::string constraintFile;
    std::cin >> constraintFile;
    auto tSet = tropic::ConstraintFactory::create(constraintFile);
    if (tSet)
    {
      entity.publish("SetTrajectory", tSet);
    }
    else
    {
      RLOG_CPP(0, "Failed to load constraint file " << constraintFile);
    }

  }, "Load constraint set from file");

  viewer.setKeyCallback('b', [&entity, &graphC](char k)
  {
    RLOG_CPP(0, "Loading PouringConstraint");

    std::shared_ptr<tropic::ConstraintSet> tSet = std::make_shared<tropic::PouringConstraint>();
    if (tSet)
    {
      entity.publish("SetTrajectory", tSet);
    }
    else
    {
      RLOG_CPP(0, "Failed to load tropic::PouringConstraint");
    }

  }, "Load constraint set from file");


  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    return;
  }

  // Start threads (if any)
  if (withGui==true)
  {
    new Dc::EventGui(&entity);
  }

  entity.initialize(graphC.getGraph());

  if (pc && (!pc->getPhysicsSimulation()))
  {
    entity.publish("RenderCommand", std::string("IK"),
                   std::string("unsetGhostMode"));
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
    entity.process();
    entity.stepTime();
    dtProcess = Timer_getSystemTime() - dtProcess;

    loopCount++;
    if (loopCount%speedUp == 0)
    {
      Timer_waitDT(entity.getDt() - dtProcess);
    }

    char text[256];
    sprintf(text, "Time: %.3f\nprocess() took %.3f",
            entity.getTime(), 1.0e3*dtProcess);
    entity.publish<std::string>("SetText", std::string(text));
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

}

/*******************************************************************************
 * Aruco marker tracking test
 ******************************************************************************/
static void testAruco()
{
#if defined (USE_OPENCV_ARUCO)

  EntityBase entity;

  CmdLineParser argP;
  double dt = 0.005;
  char xmlFileName[128] = "cAction.xml";
  char directory[128] = "config/xml/Scitos";
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);

  Rcs_addResourcePath(directory);

  entity.setDt(dt);

  RcsGraph* graph = RcsGraph_create(xmlFileName);

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  // auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  GraphicsWindow viewer(&entity);
  GraphComponent graphC(&entity, graph);
  RoboArucoComponent arucoC(&entity);

  RcsGraph_destroy(graph);

  if (argP.hasArgument("-h"))
  {
    Rcs_printResourcePath();
    KeyCatcherBase::printRegisteredKeys();
    argP.print();
    entity.print(std::cout);
    return;
  }

  EventGui::create(&entity);

  entity.initialize(graphC.getGraph());

  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());
    computeKinematics->call(graphC.getGraph());
    setRenderCommand->call();
    postUpdateGraph->call(NULL, graphC.getGraph());

    entity.process();
    dtProcess = Timer_getSystemTime() - dtProcess;

    char text[256];
    sprintf(text, "process() took %.3f msec   fps: %.3f", 1.0e3*dtProcess, arucoC.getFPS());
    entity.publish<std::string>("SetText", std::string(text));

    Timer_waitDT(entity.getDt());

    RPAUSE_DL(2);
  }

  entity.publish<>("Stop");
  entity.process();

#else

  RMSG("Aruco not compiled into library - giving up");

#endif
}

/*******************************************************************************
 * Test to speech test
 ******************************************************************************/
static void testTTS(int argc, char** argv)
{
  const double dt = 0.01;
  Dc::EntityBase entity;
  entity.subscribe("Quit", &quit);
  entity.setDt(dt);

  Dc::TTSComponent tts(&entity, -1);
  new Dc::EventGui(&entity);
  entity.publish("Start");

  while (runLoop)
  {
    entity.process();
    entity.stepTime();
    Timer_waitDT(dt);
    RLOG_CPP(1, "Time: " << entity.getTime());
  }

  entity.publish("Stop");
  entity.process();
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  int mode = 0;
  CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  switch (mode)
  {
    case 0:
      printf("1\tJoint Gui test\n");
      printf("2\tTask Gui test with IK\n");
      printf("3\tTrajectory test with IK\n");
      printf("4\tAruco marker tracking test\n");
      printf("5\tText to speech test\n");
      break;

    case 1:
      testJointGui(argc, argv);
      break;

    case 2:
      testTaskGui();
      break;

    case 3:
      testTrajectory();
      break;

    case 4:
      testAruco();
      break;

    case 5:
      testTTS(argc, argv);
      break;

    default:
      RMSG("No mode %d", mode);
  }



  if (argP.hasArgument("-h"))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    Dc::EntityBase entity;
    Dc::getHardwareComponents(entity, NULL, true);
    argP.print();
    Dc::printHardwareComponents();
  }


  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the ExampleDcComponents app\n");

  return 0;
}
