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

#include "EventSystem.h"
#include "EntityBase.h"
#include "GraphComponent.h"
#include "GraphicsWindow.h"
#include "TaskGuiComponent.h"
#include "JointGuiComponent.h"
#include "PhysicsComponent.h"
#include "IKComponent.h"
#include "DyadicMotionPlannerComponent.h"
#include "EventGui.h"
#include "ManipulationComponent.h"
#include "RemoteVisualizationComponent.h"
#include "MonitorComponent.h"
#include "BoxObjectChanger.h"
#include "StateEstimatorComponent.h"
#include "EcsHardwareComponents.h"
#include "BoxObjectModel.h"
#include "TTSComponent.h"

#if defined (WITH_HOLO)
//#include "HoloTestComponent.h"
#include "HoloComponent.h"
#endif

#if defined (WITH_INTENTION_VIS)
#include "IntentionVisualizationComponent.h"
#endif
//#include "HoloStudyDataLogger.h"
#include "TrajectoryComponent.h"
//#include "ContactVisualizer.h"


#include <ControllerBase.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_guiFactory.h>
#include <ForceDragger.h>
#include <BulletDebugNode.h>
#include <SegFaultHandler.h>
#include <WheelObjectModel.h>
#include <WheelPlannerComponent.h>

#if defined (USE_DC_ROS)
#include <VirtualFence.h>
#include <ROSSpinnerComponent.h>
#include <HoloFence.h>
#endif

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

  NamedBodyForceDragger(PhysicsBase* sim) : ForceDragger(sim)
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
 *
 ******************************************************************************/
static void boxDemo(int argc, char** argv)
{
  EntityBase entity;

  CmdLineParser argP(argc, argv);
  double dt = 0.01;
  char cfgFile[128] = "cSimpleBox.xml";
  char directory[128] = "config/xml/DexterousCooperation/BoxTurning";
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/vortex.xml";
  argP.getArgument("-f", cfgFile, "Configuration file name "
                   "(default is %s)", cfgFile);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  bool withGui = argP.hasArgument("-gui", "Task gui control");
  //  bool withAvatar = argP.hasArgument("-avatar", "Avatar connectivity");
  bool noEventGui = argP.hasArgument("-noEventGui", "Don't launch EventGui");
  bool noMonitorGui = argP.hasArgument("-noMonitorGui", "Don't launch MonitorGui");
  bool withPhysicsSim = argP.hasArgument("-physics", "Simulate physics");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step in updateGraph()");
  entity.setDt(dt);
  //entity.setTime(Timer_getSystemTime());

  Rcs_addResourcePath(directory);

  ControllerBase* controller = new ControllerBase(cfgFile);
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

  GraphicsWindow viewer(&entity, true);
  GraphComponent graphC(&entity, graph);
  std::shared_ptr<BoxObjectModel> model(new BoxObjectModel(controller, BoxStrategy5D::Box, 36, &entity));

  DyadicMotionPlannerComponent dmpc(&entity, controller, 10.0*M_PI/180.0);
  StateEstimatorComponent sec(&entity, model);
  ManipulationComponent mc(&entity, graph, model);
  IKComponent ikc(&entity, controller);
  RemoteVisualizationComponent rv(&entity, ikc.getGraph());
#if defined (WITH_HOLO)
  HoloComponent hc(&entity, "IME_VuforiaMarker");
#endif

#if defined (WITH_INTENTION_VISUALIZATION)
  IntentionVisualizationComponent<BoxObjectModel::Intention> ivc(&entity);
#endif
  BoxObjectChanger objC(&entity);

  //real sensors ,  show Gui  , auto publish, auto accept all confirmation requests
  MonitorComponent monC(&entity, graph, model, false, false, false, false); // for simulator
  monC.enableGui(!noMonitorGui);
  monC.useRealSensors(true);
  //  monC.enableAutoPublish(true);

  std::shared_ptr<TaskGuiComponent> taskGui;
  if (withGui)
  {
    taskGui = std::make_shared<TaskGuiComponent>(&entity, controller);
    taskGui->setPassive(true);
  }

  std::vector<ComponentBase*> hwc = getHardwareComponents(entity, graph);

  if (!hwc.empty())
  {
    RLOG(0, "Hardware used --> pure visualization/simulation...");
  }
  entity.registerEvent<std::string, std::vector<double> >("SetCompliance");

  std::vector<double> comp(6, 0.0);
  comp[0] = 5000.0;
  comp[1] = 5000.0;
  comp[2] = 5000.0;
  comp[3] = 300.0;
  comp[4] = 300.0;
  comp[5] = 300.0;


  entity.publish("SetCompliance", std::string("right"), comp);
  entity.publish("SetCompliance", std::string("left"), comp);


  if (withPhysicsSim)
  {
    if (hwc.empty())
    {
      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        JNT->ctrlType = RCSJOINT_CTRL_POSITION;
      }

      PhysicsComponent* pc = new PhysicsComponent(&entity, graph, physicsEngine, physicsCfg, !seqSim);
      osg::ref_ptr<NamedBodyForceDragger> dragger = new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
      hwc.push_back(pc);
    }
  }

  viewer.setKeyCallback('u',
                        [&entity](char k)
  {
    entity.publish("Confirmation", 0, 10000);
  },
  "Override Monitor Confirmation triggers"
                       );
  viewer.setKeyCallback('o',
                        [&entity](char k)
  {
    entity.publish("Intention", (int) BoxObjectModel::Intention::ROTATE_RIGHT, 10000);
  },
  "Broadcast intention to rotate object RIGHT"
                       );
  viewer.setKeyCallback('i',
                        [&entity](char k)
  {
    entity.publish("Intention", (int) BoxObjectModel::Intention::ROTATE_LEFT, 10000);
  },
  "Broadcast intention to rotate object LEFT"
                       );

  viewer.setKeyCallback('d',
                        [&entity](char k)
  {
    entity.publish("ResetErrorFlag");
  },
  "Reseting IME failure flag"
                       );
  viewer.setKeyCallback('T',
                        [&entity](char k)
  {
    entity.publish("Tare");
    RLOG(0, "'Tare' command sent.");
  },
  "Taring sensors."
                       );
  viewer.setKeyCallback('z', [&entity](char k)
  {
    entity.publish("ToggleObjectShape");
  }, "Toggle object shape between box, cylinder and l-shape");

  viewer.setKeyCallback('h', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 5000.0;
    comp[1] = 5000.0;
    comp[2] = 5000.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('j', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 900.0;
    comp[1] = 900.0;
    comp[2] = 1500.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('k', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 300.0;
    comp[1] = 300.0;
    comp[2] = 900.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('l', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 2000.0;
    comp[1] = 2000.0;
    comp[2] = 2000.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('J', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 150.0;
    comp[1] = 150.0;
    comp[2] = 900.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('K', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 150.0;
    comp[1] = 150.0;
    comp[2] = 600.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('L', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 5000.0;
    comp[1] = 5000.0;
    comp[2] = 5000.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 20.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('b', [&entity](char k)
  {
    entity.publish("StartLogging");

  }, "Start Logging");

  viewer.setKeyCallback('B', [&entity](char k)
  {
    entity.publish("StopLogging");

  }, "Stop Logging");


  //HoloStudyDataLogger g1(&entity);

  delete controller;

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    fprintf(stderr, "Mode 1: Box demo: \n");
    fprintf(stderr, "See README.md for command line\n");
    return;
  }


  entity.publish("ChangeObjectShape", std::string("Box"));

  // Start threads (if any)
  if (noEventGui == false)
  {
    EventGui::create(&entity);
  }
  entity.publish("SetTTC", 6.0);

#if 0
  // Start threads (if any)
  entity.call("Start");

  // Initialization sequence
  //RPAUSE_MSG("Hit enter to start");
  entity.call("Render");

  // RPAUSE_MSG("Hit enter to update graph");
  updateGraph->call(graphC.getGraph());
  computeKinematics->call(graphC.getGraph());
  entity.call<const RcsGraph*>("InitFromState", graphC.getGraph());
  entity.call("Render");

  //RPAUSE_MSG("Hit enter to enable commands");
  entity.call("EnableCommands");
#else
  // Start threads (if any)
  RPAUSE_MSG_DL(1, "Start");
  entity.publish("Start");
  int nIter = entity.processUntilEmpty(10);
  RLOG(1, "Start took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  // Initialization sequence
  // RPAUSE_MSG_DL(1, "Render");
  entity.publish("Render");
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "Render took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  RPAUSE_MSG_DL(1, "updateGraph");
  updateGraph->call(graphC.getGraph());
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "updateGraph took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  computeKinematics->call(graphC.getGraph());
  entity.processUntilEmpty(10);
  entity.publish("Render");
  entity.processUntilEmpty(10);

  RPAUSE_MSG_DL(1, "updateGraph");
  updateGraph->call(graphC.getGraph());
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "updateGraph took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  computeKinematics->call(graphC.getGraph());
  entity.processUntilEmpty(10);

  entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
  entity.publish("Render");
  entity.processUntilEmpty(10);

  RPAUSE_MSG_DL(1, "EnableCommands");
  entity.publish("EnableCommands");
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "InitFromState++ took %d process() calls, queue is %zu",
       nIter, entity.queueSize());
  RPAUSE_MSG_DL(1, "Enter runLoop");
#endif

  // prepare transforms
#if defined (WITH_HOLO)
  entity.publish("HoloTrackTransform", std::string("PowerGrasp_R"), GraphType::Desired);
  entity.publish("HoloTrackTransform", std::string("PowerGrasp_L"), GraphType::Desired);
#endif
  //RPAUSE_MSG("Hit enter to start run loop");

  int count = 0;
  int aboveLoopThresCount = 0;

  while (runLoop)
  {
    double loopStartTime = Timer_getSystemTime();

    entity.stepTime();

    //desired graph: ikc.getGraph()
    //current graph: graphC.getGraph()

    //    if (withPhysicsSim)
    //    {
    updateGraph->call(graphC.getGraph());
    //    }
    //    else
    //    {
    //      updateGraph->call(ikc.getGraph());
    //    }

    // This updates the TaskGui and filters the commands
    //    if (withPhysicsSim)
    //    {
    computeKinematics->call(graphC.getGraph());
    //    }
    //    else
    //    {
    //      computeKinematics->call(ikc.getGraph());
    //    }
    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());

    //TODO: in case we need this later
    //updateState->call()

    if (!entity.getTimeFrozen())
    {
      // This updates the TaskGui and filters the commands
      computeTrajectory->call(ikc.getGraph());

      // This computes the IK with the gui's inputs into the IKComponents
      // internal q_des (a, x -> q)
      setTaskCommand->call(dmpc.getActivationPtr(), dmpc.getTaskCommandPtr());

      // This distributes the IKComponents internal command to all motor components
      setJointCommand->call(ikc.getJointCommandPtr());   // motors <- q
    }

    // This triggers graphics updates in some components
    setRenderCommand->call();

    double dtProcess1 = Timer_getSystemTime();
    entity.process();
    double dtProcess2 = Timer_getSystemTime();

    double dtProcess = Timer_getSystemTime() - loopStartTime;

    count++;
    if (dtProcess > 0.01)
    {
      aboveLoopThresCount++;
    }

    if (count % 100 == 0)
    {
      //      const size_t rhIdx = controller->getTaskArrayIndex("XYZ_R");
      //      const size_t lhIdx = controller->getTaskArrayIndex("XYZ_L");


      char text[256];
      sprintf(text, "process() took %6.3f (%6.3f, %6.3f) msec",
              1.0e3*dtProcess,  1.0e3*(dtProcess1-loopStartTime), 1.0e3*(dtProcess2-dtProcess1));
      entity.publish<std::string>("SetTextLine", std::string(text), 3);
    }
    Timer_waitDT(entity.getDt() - dtProcess);
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }
}

/*******************************************************************************
 * This is the cool compliant motor cycle wheel demo
 *
 * Move platform back to home:
 *   !!! For moving back, please use LBR position control mode !!!
 *   bin/InitWheelBot -ime -vicon
 *
 * Move arms and fingers into home pose:
 *   bin/InitWheelBot -lbr1 -lbr2 -sdh1 -sdh2
 *
 * Run demo:
 *   bin/TestRobo -m 7 -ime -ief -lbr1 -lbr2 -sdh1 -sdh2 -fts1 -fts2 -vicon
 *
 * Gui operation:
 *   Tare -> Move to start -> Engage -> Close Hands -> Auto Evaluate
 *
 * Trouble shooting:
 *   CANBlaBla cannot be found: export LD_LIBRARY_PATH=lib:$LD_LIBRARY_PATH
 *   VICON PC locked: see DexterousCooperation/Support/Computing/vicon-pc
 ******************************************************************************/
static void wheelDemo(int argc, char** argv)
{
  EntityBase entity;

  CmdLineParser argP(argc, argv);
  double dt = 0.01;
  char cfgFile[128] = "cSimpleWheel.xml";
  char directory[128] = "config/xml/DexterousCooperation/WheelTurning";
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  argP.getArgument("-f", cfgFile, "Configuration file name "
                   "(default is %s)", cfgFile);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  bool withGui = argP.hasArgument("-gui", "Task gui control");
  //  bool withAvatar = argP.hasArgument("-avatar", "Avatar connectivity");
  bool noEventGui = argP.hasArgument("-noEventGui", "Don't launch EventGui");
  bool noMonitorGui = argP.hasArgument("-noMonitorGui", "Don't launch MonitorGui");
  bool withPhysicsSim = argP.hasArgument("-physics", "Simulate physics");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation step in updateGraph()");
  bool zigzag = argP.hasArgument("-zigzag", "Zig-zag trajectory");
  bool speak = argP.hasArgument("-speak", "Enable text to speech");

  entity.setDt(dt);
  //entity.setTime(Timer_getSystemTime());
  Rcs_addResourcePath(directory);

  ControllerBase* controller = new ControllerBase(cfgFile);
  RcsGraph* graph = controller->getGraph();

  auto updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  auto postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");
  auto computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  auto computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  auto setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  auto setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  auto setRenderCommand = entity.registerEvent<>("Render");
  auto setComplianceCommand = entity.registerEvent<std::vector<double>>("SetComplianceCommand");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  GraphicsWindow viewer(&entity, true);
  viewer.setCameraTransform(0.709573, 5.297887, 3.003564,   0.400872, -0.086577, -1.813365);
  GraphComponent graphC(&entity, graph);
  std::shared_ptr<WheelObjectModel> model(new WheelObjectModel(controller, &entity));

  //HoloComponent hc(&entity, "ViconVuforia");
  //IntentionVisualizationComponent<WheelObjectModel::Intention> ivc(&entity);
  WheelPlannerComponent dmpc(&entity, controller, !zigzag, 1.0);
  StateEstimatorComponent sec(&entity, model);
  ManipulationComponent mc(&entity, graph, model);
  IKComponent ikc(&entity, controller);

  //  RemoteVisualizationComponent rv(&entity, ikc.getGraph());
  //  BoxObjectChanger objC(&entity);

  //real sensors ,  show Gui  , auto publish, auto accept all confirmation requests
  MonitorComponent monC(&entity, graph, model, false, false, false, false); // for simulator
  monC.enableGui(!noMonitorGui);
  monC.useRealSensors(true);
  //  monC.enableAutoPublish(true);

  std::shared_ptr<TaskGuiComponent> taskGui;
  if (withGui)
  {
    taskGui = std::make_shared<TaskGuiComponent>(&entity, controller);
    taskGui->setPassive(true);
  }

  std::vector<ComponentBase*> hwc = getHardwareComponents(entity, graph);

  if (!hwc.empty())
  {
    RLOG(0, "Hardware used --> pure visualization/simulation...");
  }
  entity.registerEvent<std::string, std::vector<double> >("SetCompliance");


  std::vector<double> comp(6, 0.0);
  comp[0] = 5000.0;
  comp[1] = 5000.0;
  comp[2] = 5000.0;
  comp[3] = 300.0;
  comp[4] = 300.0;
  comp[5] = 300.0;


  entity.publish("SetCompliance", std::string("right"), comp);
  entity.publish("SetCompliance", std::string("left"), comp);



  if (withPhysicsSim)
  {
    if (hwc.empty())
    {
      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        JNT->ctrlType = RCSJOINT_CTRL_POSITION;
      }

      PhysicsComponent* pc = new PhysicsComponent(&entity, graph, physicsEngine, physicsCfg, !seqSim);
      osg::ref_ptr<NamedBodyForceDragger> dragger = new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
      hwc.push_back(pc);
    }
  }


#if defined (USE_DC_ROS)

  HoloFence* hFence = NULL;

  if (argP.hasArgument("-ros"))
  {
    hwc.push_back(new VirtualFence(&entity, &viewer, "safety_zone_visualization", "/safety_training"));
    hwc.push_back(new ROSSpinnerComponent(&entity, 1));
    hFence = new HoloFence(&entity, "safety_zone_visualization", "/safety_training");
    hwc.push_back(hFence);
  }
#endif


  //  GraphLogger g2(&entity, "LogBoxBotDesiredState.dat", false);

  if (speak)
  {
    hwc.push_back(new TTSComponent(&entity));
  }

  viewer.setKeyCallback('u',
                        [&entity](char k)
  {
    entity.publish("Confirmation", 0, 10000);
  },
  "Override Monitor Confirmation triggers"
                       );
  viewer.setKeyCallback('o',
                        [&entity](char k)
  {
    entity.publish("Intention", (int) BoxObjectModel::Intention::ROTATE_RIGHT, 10000);
  },
  "Broadcast intention to rotate object RIGHT"
                       );
  viewer.setKeyCallback('i',
                        [&entity](char k)
  {
    entity.publish("Intention", (int) BoxObjectModel::Intention::ROTATE_LEFT, 10000);
  },
  "Broadcast intention to rotate object LEFT"
                       );

  viewer.setKeyCallback('d',
                        [&entity](char k)
  {
    entity.publish("ResetErrorFlag");
  },
  "Reseting IME failure flag"
                       );
  viewer.setKeyCallback('T',
                        [&entity](char k)
  {
    entity.publish("Tare");
    RLOG(0, "'Tare' command sent.");
  },
  "Taring sensors."
                       );
  viewer.setKeyCallback('z', [&entity](char k)
  {
    entity.publish("ToggleObjectShape");
  }, "Toggle object shape between box, cylinder and l-shape");

  //  viewer.setKeyCallback('K', [&entity](char k)
  //  {
  //    std::vector<double> comp(6, 0.0);
  //    comp[0] = 300.0;
  //    comp[1] = 300.0;
  //    comp[2] = 900.0;
  //    comp[3] = 300.0;
  //    comp[4] = 300.0;
  //    comp[5] = 300.0;
  //
  //    entity.publish("SetCompliance", std::string("left"), comp);
  //    entity.publish("SetCompliance", std::string("right"), comp);
  //  }, "Reset compliance for both arms");

  viewer.setKeyCallback('k', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 300.0;
    comp[1] = 300.0;
    comp[2] = 900.0;
    comp[3] = 30.0;
    comp[4] = 30.0;
    comp[5] = 30.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('j', [&entity](char k)
  {
    std::vector<double> comp(6, 0.0);
    comp[0] = 5000.0;
    comp[1] = 5000.0;
    comp[2] = 5000.0;
    comp[3] = 300.0;
    comp[4] = 300.0;
    comp[5] = 300.0;

    entity.publish("SetCompliance", std::string("left"), comp);
    entity.publish("SetCompliance", std::string("right"), comp);
  }, "Reset compliance for both arms");

  viewer.setKeyCallback('e', [&entity](char k)
  {
    entity.publish("Engage", 6.0);

  }, "Engage hands");

  viewer.setKeyCallback('E', [&entity](char k)
  {
    entity.publish("Disengage", 6.0);

  }, "Disengage hands");
  viewer.setKeyCallback('b', [&entity](char k)
  {
    entity.publish("StartLogging");

  }, "Start Logging");

  viewer.setKeyCallback('B', [&entity](char k)
  {
    entity.publish("StopLogging");

  }, "Stop Logging");

  //HoloStudyDataLogger g1(&entity);

  delete controller;

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    fprintf(stderr, "Mode 2: Wheel demo: \n");
    fprintf(stderr, "See README.md for command line\n");
    return;
  }

  // Start threads (if any)
  RPAUSE_MSG_DL(1, "Start");
  entity.publish("Start");
  int nIter = entity.processUntilEmpty(10);
  RLOG(1, "Start took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  // Initialization sequence
  // RPAUSE_MSG_DL(1, "Render");
  entity.publish("Render");
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "Render took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  RPAUSE_MSG_DL(1, "updateGraph");
  updateGraph->call(graphC.getGraph());
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "updateGraph took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  computeKinematics->call(graphC.getGraph());
  entity.processUntilEmpty(10);
  entity.publish("Render");
  entity.processUntilEmpty(10);




  RPAUSE_MSG_DL(1, "updateGraph");
  updateGraph->call(graphC.getGraph());
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "updateGraph took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  computeKinematics->call(graphC.getGraph());
  entity.processUntilEmpty(10);

  entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
  //if (hwc.empty())
  {
    //entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));
  }

  entity.publish("Render");
  entity.processUntilEmpty(10);

#if defined (USE_DC_ROS)

  if (hFence)
  {
    const RcsBody* vuforia = RcsGraph_getBodyByName(graphC.getGraph(), "ViconVuforia");
    hFence->setOrigin(vuforia->A_BI);
  }
#endif

  RPAUSE_MSG_DL(1, "EnableCommands");
  entity.publish("EnableCommands");
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "InitFromState++ took %d process() calls, queue is %zu",
       nIter, entity.queueSize());
  RPAUSE_MSG_DL(1, "Enter runLoop");

  if (noEventGui == false)
  {
    EventGui::create(&entity);
  }

  entity.publish("SetTTC", 6.0);

  int count = 0;
  int aboveLoopThresCount = 0;

  // tell the user in advance if a regrasp will happen
  entity.subscribe<double>("NotifyRegraspLeft", [&entity](double t)
  {
    RLOG(0, "NotifyRegraspLeft: %5.2f", t);
    // Tell Hololens users a regrasp will happen
    entity.publish("HoloTTS", std::string("Hold the wheel steady!"));
  });
  // tell the user in advance if a regrasp will happen
  entity.subscribe<double>("NotifyRegraspRight", [&entity](double t)
  {
    RLOG(0, "NotifyRegraspLeft: %5.2f", t);
    // Tell Hololens users a regrasp will happen
    entity.publish("HoloTTS", std::string("Hold the wheel steady!"));
  });

  while (runLoop)
  {
    double loopStartTime = Timer_getSystemTime();

    entity.stepTime();

    // Step all systems
    updateGraph->call(graphC.getGraph());
    computeKinematics->call(graphC.getGraph());
    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());
    computeTrajectory->call(ikc.getGraph());
    setTaskCommand->call(dmpc.getActivationPtr(), dmpc.getTaskCommandPtr());
    setJointCommand->call(ikc.getJointCommandPtr());
    std::vector<double> wrench = dmpc.getComplianceWrench();
    setComplianceCommand->call(wrench);
    setRenderCommand->call();

    // Compute timings
    double dtProcess1 = Timer_getSystemTime()-loopStartTime;
    entity.process();
    double dtProcess2 = Timer_getSystemTime()-loopStartTime;
    count++;

    if (dtProcess2 > entity.getDt())
    {
      aboveLoopThresCount++;
    }

    // Graphics window text message update
    char text[256];
    sprintf(text, "Time: %.3f   Step took %6.1f (systems: %.1f, process: %.1f) msec",
            entity.getTime(), 1.0e3*dtProcess2,  1.0e3*dtProcess1, 1.0e3*(dtProcess2-dtProcess1));
    entity.publish<std::string>("SetTextLine", std::string(text), 3);

    snprintf(text, 256, "Compliance: rpos: %.0f rori: %.0f lpos: %.0f lori: %.0f",
             wrench[0], wrench[3], wrench[6], wrench[9]);
    entity.publish("SetTextLine", std::string(text), 5);

    // Wait for next iteration
    Timer_waitDT(entity.getDt() - dtProcess2);
  }

  entity.publish<>("Stop");
  entity.process();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

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

  Rcs_addResourcePath("config");

  switch (mode)
  {
    case 0:
      break;

    case 1:
      boxDemo(argc, argv);
      break;

    case 2:
      wheelDemo(argc, argv);
      break;

    default:
      RMSG("No mode %d", mode);
  }



  if (argP.hasArgument("-h") || (mode==0))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();

    fprintf(stderr, "Test modes:\n");
    fprintf(stderr, "1. Box turning demo\n");
    fprintf(stderr, "2. Wheel turning demo\n");
  }


  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
