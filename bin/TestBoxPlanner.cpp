/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

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
#include "EcsHardwareComponents.h"
#include "BoxObjectChanger.h"
#include "TrajectoryComponent.h"

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
 * Planning and trajectory control including IK, through EventGui
 ******************************************************************************/
static void testBoxPlanner(const char* cfgFile)
{
  CmdLineParser argP;
  double dt = 0.01, dPhi=30.0, ttc = 6.0, speedUp = 1.0;
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default is \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default is %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-dPhi", &dPhi, "Angular discretization in degrees (default"
                   " is %f)", dPhi);
  bool withGui = argP.hasArgument("-gui", "Task gui control");
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
  bool startViewerWithStartEvent = argP.hasArgument("-h");

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

  EntityBase entity;
  entity.setDt(dt);
  if (pause)
  {
    entity.call("TogglePause");
  }

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

  GraphicsWindow viewer(&entity, startViewerWithStartEvent, seqViewer, simpleGraphics);
  GraphComponent graphC(&entity, graph);
  DyadicMotionPlannerComponent dmpc(&entity, controller, dPhi*M_PI/180.0, !zigzag);
  IKComponent ikc(&entity, controller);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);

  //RebaComponent rebaC(&entity, "cLinda.xml");
  BoxObjectChanger objC(&entity);

  std::shared_ptr<TaskGuiComponent> taskGui;
  if (withGui==true)
  {
    taskGui = std::make_shared<TaskGuiComponent>(&entity, controller);
    taskGui->setPassive(true);
  }

  std::vector<ComponentBase*> hwc = getHardwareComponents(entity, graph);

  if (hwc.empty())
  {
    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    PhysicsComponent* pc = new PhysicsComponent(&entity, graph, physicsEngine,
                                                physicsCfg, !seqSim);

    ttc = 1.0;
    argP.getArgument("-ttc", &ttc, "Transition time (default is %f)", ttc);
    argP.getArgument("-speedUp", &speedUp, "SpeedUp (default is %f)", speedUp);

    if (pc->getPhysicsSimulation())
    {
      osg::ref_ptr<NamedBodyForceDragger> dragger =
        new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
    }
    else
    {
      graphC.setEnableRender(false);
      ikc.renderSolidModel();
    }

    if (seqSim && seqViewer)
    {
#if defined (USE_BULLET)
      osg::ref_ptr<BulletDebugNode> btNd =
        new BulletDebugNode(pc->getPhysicsSimulation(), viewer.getFrameMtx());
      viewer.add(btNd.get());
#endif
    }

    hwc.push_back(pc);
  }
  else
  {
    ttc = 6.0;
    argP.getArgument("-ttc", &ttc, "Transition time (default is %f)", ttc);
    RCHECK(ttc >= 6.0);
  }


  delete controller;

  viewer.setKeyCallback('d', [&entity](char k)
  {
    entity.publish("ResetErrorFlag");
  }, "Reseting IME failure flag");

  viewer.setKeyCallback('z', [&entity](char k)
  {
    entity.publish("ToggleObjectShape");
  }, "Toggle object shape between box, cylinder and l-shape");

  viewer.setKeyCallback('b', [&entity](char k)
  {
    entity.publish("StopMovement", 1.9);
  }, "Stop movement");

  viewer.setKeyCallback('m', [&entity, &ttc](char k)
  {
    entity.publish("MoveToStartPose", ttc);
  }, "Move to start pose");

  viewer.setKeyCallback('a', [&entity, &ttc](char k)
  {
    entity.publish("PlanToAngleAndExecute", 3.1415, ttc);
  }, "Plan rotation to +PI");

  viewer.setKeyCallback('A', [&entity, &ttc](char k)
  {
    entity.publish("PlanToAngleAndExecute", -3.1415, ttc);
  }, "Plan rotation to -PI");

  viewer.setKeyCallback('e', [&dmpc](char k)
  {
    auto moveSet = dmpc.createExampleTrajectory();
    moveSet->toXML("ConstraintSet.xml");
  }, "Write example trajectory to file");

  if (argP.hasArgument("-h"))
  {
    entity.print(std::cout);
    for (size_t i=0; i<hwc.size(); ++i)
    {
      delete hwc[i];
    }
    return;
  }




  if (noEventGui == false)
  {
    EventGui::create(&entity);
  }

  RPAUSE_MSG_DL(1, "ChangeObjectShape");
  entity.publish<std::string>("ChangeObjectShape", "Box");
  int nIter = entity.processUntilEmpty(10);
  RLOG(0, "ChangeObjectShape took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  // Start threads (if any)
  RPAUSE_MSG_DL(1, "Start");
  entity.publish("Start");
  nIter = entity.processUntilEmpty(10);
  RLOG(0, "Start took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  // Initialization sequence
  RPAUSE_MSG_DL(1, "Render");
  entity.publish("Render");
  nIter = entity.processUntilEmpty(10);
  RLOG(0, "Render took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  RPAUSE_MSG_DL(1, "updateGraph");
  updateGraph->call(graphC.getGraph());
  nIter = entity.processUntilEmpty(10);
  RLOG(0, "updateGraph took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  computeKinematics->call(graphC.getGraph());
  entity.processUntilEmpty(10);

  entity.publish<const RcsGraph*>("InitFromState", graphC.getGraph());
  entity.publish("Render");
  entity.publish("EnableCommands");
  nIter = entity.processUntilEmpty(10);
  RLOG(0, "InitFromState++ took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());

    // This updates the TaskGui and filters the commands
    computeKinematics->call(graphC.getGraph());

    // This updates the TaskGui and filters the commands
    computeTrajectory->call(ikc.getGraph());

    // This computes the IK with the planner's inputs into the IKComponents
    // internal q_des:  a, x -> q
    setTaskCommand->call(dmpc.getActivationPtr(), dmpc.getTaskCommandPtr());

    // Distribute the IKComponents internal command to all motor components
    setJointCommand->call(ikc.getJointCommandPtr());   // motors <- q

    // This triggers graphics updates in some components
    setRenderCommand->call();

    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());

    entity.process();
    dtProcess = Timer_getSystemTime() - dtProcess;

    // char text[256];
    // sprintf(text, "process() took %.3f msec\nREBA: %d %.2f", 1.0e3*dtProcess,
    //         rebaC.getRebaScore(), rebaC.getAverageRebaScore());
    // entity.publish<std::string>("SetText", std::string(text));

    Timer_waitDT(entity.getDt()/speedUp - dtProcess);
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
  char xmlFileName[128] = "cSimpleBox.xml";
  char directory[128] = "config/xml/DexterousCooperation/BoxTurning";
  int mode = 0;
  CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(directory);

  switch (mode)
  {
    case 0:
      testBoxPlanner(xmlFileName);
      break;

    default:
      RMSG("No mode %d", mode);
  }



  if (argP.hasArgument("-h"))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();

    fprintf(stderr, "Real robot box demo: bin/TestBoxPlanner -lbr1 -lbr2 -ime -ief -sdh1 -sdh2 -fts1 -fts2 -vic -ttc 6\n");
    fprintf(stderr, "1. Press d twice in viewer window to enable IME\n");
  }


  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the TestBoxPlanner app\n");

  return 0;
}
