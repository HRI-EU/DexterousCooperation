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

/*******************************************************************************

  \todo:

  - MG: Enable planning with the human
  - MG: Scale null space gradients with dt
  - MG: Pause-free wheel motion with 1-via points
  - MG: Is there any way to avoid calculating distances twice? We need it to
        calculate the Jacobian orientation and the delta x. Maybe with
        computeCollisionModel() and getCollisionModel()? Could be an extra step
        in solving. But maybe outside class, since we want to compute null
        space grasients as well. Many options, think about it.

*******************************************************************************/

#include "ExampleWheelPlanner.h"

#include <EventGui.h>
#include <EcsHardwareComponents.h>
#include <WheelStrategy7D.h>
#include <WheelConstraint.h>
#include <WheelNode.h>
#include <TTSComponent.h>

#include <ExampleFactory.h>
#include <PhysicsFactory.h>
#include <ForceDragger.h>
#include <BulletDebugNode.h>
#include <BodyPointDragger.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_kinematics.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>



namespace Dc
{

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger : public Rcs::ForceDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim) : Rcs::ForceDragger(sim)
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
RCS_REGISTER_EXAMPLE(ExampleWheelPlanner, "Dexterous Cooperation", "Wheel planner");

ExampleWheelPlanner::ExampleWheelPlanner(int argc, char** argv) :
  ExampleBase(argc, argv), entity()
{
  Rcs::CmdLineParser argP(argc, argv);

  speedUp = 1;
  loopCount = 0;
  queueSize = 0;
  dt = 0.01;
  ttc = 8.0;
  alpha = 0.02;
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
  startViewerWithStartEvent = false;
  valgrind = false;

  controller = NULL;

  updateGraph = NULL;;
  postUpdateGraph = NULL;;
  computeKinematics = NULL;;
  computeTrajectory = NULL;;
  setTaskCommand = NULL;;
  setJointCommand = NULL;;
  setRenderCommand = NULL;;
  setComplianceCommand = NULL;;
}

ExampleWheelPlanner::~ExampleWheelPlanner()
{
  Rcs_removeResourcePath(directory.c_str());
}

bool ExampleWheelPlanner::initParameters()
{
  directory = "config/xml/DexterousCooperation/WheelTurning";
  cfgFile = "cSimpleWheel.xml";
  physicsEngine = "Bullet";
  physicsCfg = "config/physics/physics.xml";

  return true;
}

bool ExampleWheelPlanner::parseArgs(Rcs::CmdLineParser* argP)
{
  argP->getArgument("-f", &cfgFile, "Configuration file name "
                    "(default is %s)", cfgFile.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is %s)", directory.c_str());
  argP->getArgument("-physicsEngine", &physicsEngine,
                    "Physics engine (default: \"%s\")", physicsEngine.c_str());
  argP->getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)",
                    speedUp);
  argP->getArgument("-physics_config", &physicsCfg, "Configuration file name"
                    " for physics (default: %s)", physicsCfg.c_str());
  argP->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP->getArgument("-ttc", &ttc, "Transition time (default: %f)", ttc);
  argP->getArgument("-alpha", &alpha, "Null space scaler (default: %f)", alpha);
  argP->getArgument("-noEventGui", &noEventGui, "Don't launch EventGui");
  argP->getArgument("-nospeed", &noSpeedCheck, "No speed limit checks");
  argP->getArgument("-nojl", &noJointCheck, "Disable joint limit checks");
  argP->getArgument("-nocoll", &noCollCheck, "Disable collision checks");
  argP->getArgument("-sequentialPhysics", &seqSim, "Physics simulation "
                    "step in updateGraph()");
  argP->getArgument("-sequentialViewer", &seqViewer, "Viewer frame call "
                    "in \"Render\" event");
  argP->getArgument("-zigzag", &zigzag, "Zig-zag trajectory");
  argP->getArgument("-pause", &pause, "Pause after each process() call");
  argP->getArgument("-sync", &sync, "Run as sequential as possible");
  argP->getArgument("-noLimits", &noLimits, "Ignore joint, speed and "
                    "collision limits");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without "
                    "shadows and anti-aliasing");
  argP->getArgument("-h", &startViewerWithStartEvent);
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");

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

  return true;
}

bool ExampleWheelPlanner::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());
  controller = std::make_unique<Rcs::ControllerBase>(cfgFile);

  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
  setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  setRenderCommand = entity.registerEvent<>("Render");
  setComplianceCommand = entity.registerEvent<std::vector<double>>("SetComplianceCommand");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");

  entity.setDt(dt);

  if (pause)
  {
    entity.call("TogglePause");
  }


  graphC = std::make_unique<GraphComponent>(&entity, controller->getGraph());
  dmpc = std::make_unique<WheelPlannerComponent>(&entity, controller.get(), !zigzag);
  ikc = std::make_unique<IKComponent>(&entity, controller.get());
  ikc->setSpeedLimitCheck(!noSpeedCheck);
  ikc->setJointLimitCheck(!noJointCheck);
  ikc->setCollisionCheck(!noCollCheck);
  ikc->setAlpha(alpha);

  std::vector<ComponentBase*> hwc = getHardwareComponents(entity, controller->getGraph());

  // If no robot components are added, we add a simulator component
  if (hwc.empty())
  {
    // In that case, we change to position control
    RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    pc = std::make_unique<Dc::PhysicsComponent>(&entity, controller->getGraph(),
                                                physicsEngine.c_str(),
                                                physicsCfg.c_str(), !seqSim);
  }

  // Put hardware components into vector of unique pointers so that we don't
  // need to take care about their deletion
  for (size_t i = 0; i < hwc.size(); ++i)
  {
    hwComponents.push_back(std::unique_ptr<Dc::ComponentBase>(hwc[i]));
  }

  return true;
}

bool ExampleWheelPlanner::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  if (noEventGui == false)
  {
    new Dc::EventGui(&entity);
  }

  return true;
}

bool ExampleWheelPlanner::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

  viewer = std::make_unique<Dc::GraphicsWindow>(&entity, startViewerWithStartEvent,
                                                seqViewer, simpleGraphics);

  viewer->setKeyCallback('d', [this](char k)
  {
    RLOG(0, "Reseting IME failure flag");
    entity.publish("ResetErrorFlag");
  }, "Reseting IME failure flag");

  viewer->setKeyCallback('m', [this](char k)
  {
    RLOG(0, "Move robot to initial pose");
    entity.publish("InitializeMovement", ttc);
  }, "Move robot to initial pose");

  viewer->setKeyCallback('a', [this](char k)
  {
    RLOG(0, "Plan and perform sequence to [0 0]");
    entity.publish("PlanToFlipAndExecute", 0, 0);
  }, "Plan and perform sequence to [0 0]");

  viewer->setKeyCallback('A', [this](char k)
  {
    RLOG(0, "Plan and perform sequence to [0 -6]");
    entity.publish("PlanToFlipAndExecute", 0, -6);
  }, "Plan and perform sequence to [0 -6]");

  viewer->setKeyCallback('b', [this](char k)
  {
    RLOG(0, "Plan and perform sequence to [11 -3]");
    entity.publish("PlanToFlipAndExecute", 11, -3);
  }, "Plan and perform sequence to [11 -3]");

  viewer->setKeyCallback('e', [this](char k)
  {
    RLOG(0, "Launch event gui");
    new EventGui(&entity);
  }, "Launch event gui");



  if (pc && pc->getPhysicsSimulation() && viewer)
  {
    osg::ref_ptr<NamedBodyForceDragger> dragger =
      new NamedBodyForceDragger(pc->getPhysicsSimulation());

    viewer->add(dragger.get());

    if (seqSim && seqViewer)
    {
#if defined (USE_BULLET)
      osg::ref_ptr<BulletDebugNode> btNd =
        new BulletDebugNode(pc->getPhysicsSimulation(), viewer->getFrameMtx());
      viewer->add(btNd.get());
#endif
    }
  }

  return true;
}

void ExampleWheelPlanner::run()
{
  entity.initialize(graphC->getGraph());

  if (viewer)
  {
    Rcs::GraphNode* gn = viewer->getGraphNodeById("Physics");
    if (gn)
    {
      Rcs::BodyNode* anchor = gn->getBodyNode("WheelPole");
      if (anchor)
      {
        anchor->addChild(new Dc::WheelNode(dmpc->getStrategy()));
        entity.publish("Render");
        entity.processUntilEmpty(10);
      }
    }
  }


  while (runLoop)
  {
    step();
  }

  entity.publish<>("Stop");
  entity.process();
}

void ExampleWheelPlanner::step()
{
  double dtProcess = Timer_getSystemTime();
  updateGraph->call(graphC->getGraph());

  // This updates the TaskGui and filters the commands
  computeKinematics->call(graphC->getGraph());

  postUpdateGraph->call(ikc->getGraph(), graphC->getGraph());

  // This updates the TaskGui and filters the commands
  computeTrajectory->call(ikc->getGraph());

  // This computes the IK with the planner's inputs into the IKComponents
  // internal q_des:  a, x -> q
  setTaskCommand->call(dmpc->getActivationPtr(), dmpc->getTaskCommandPtr());

  // This is called right before sending the commands
  // checkEmergencyStop->call();

  // Distribute the IKComponents internal command to all motor components
  setJointCommand->call(ikc->getJointCommandPtr());   // motors <- q

  // Compute the compliance wrench depending on the task space velocities
  // of the hand-wheel relative tasks.
  auto wrench = dmpc->getComplianceWrench();
  setComplianceCommand->call(wrench);

  // This triggers graphics updates in some components
  setRenderCommand->call();

  queueSize = std::max(entity.queueSize(), queueSize);

  entity.process();
  entity.stepTime();

  dtProcess = Timer_getSystemTime() - dtProcess;


  char text[256];
  snprintf(text, 256, "Time: %.3f   dt: %.1f msec (< %.1f msec)   queue: %zu",
           entity.getTime(), 1.0e3 * dtProcess, 1.0e3 * entity.getDt(), queueSize);
  entity.publish("SetTextLine", std::string(text), 1);

  snprintf(text, 256, "Motion time: %5.3f\nState: ",
           dmpc->getMotionEndTime());
  std::vector<int> searchState = dmpc->getState();
  for (size_t i = 0; i < searchState.size(); ++i)
  {
    char a[8];
    snprintf(a, 8, "%d ", (int)searchState[i]);
    strcat(text, a);
  }

  entity.publish("SetTextLine", std::string(text), 2);

  snprintf(text, 256, "Compliance: rpos: %.0f rori: %.0f lpos: %.0f lori: %.0f",
           wrench[0], wrench[3], wrench[6], wrench[9]);
  entity.publish("SetTextLine", std::string(text), 3);


  loopCount++;
  if (loopCount % speedUp == 0)
  {
    Timer_waitDT(entity.getDt() - dtProcess);
  }
}



/*******************************************************************************
 * Test to speech test
 ******************************************************************************/
class ExampleTTS : public Rcs::ExampleBase
{
public:
  Rcs::AsyncWidget* tg;
  ExampleTTS(int argc, char** argv) : ExampleBase(argc, argv)
  {
    tg = NULL;
  }

  ~ExampleTTS()
  {
    delete tg;
  }

  void quit()
  {
    RLOG(0, "quit() called");
    runLoop = false;
  }

  void run()
  {
    const double dt = 0.01;
    Dc::EntityBase entity;
    entity.setDt(dt);
    entity.subscribe("Quit", &ExampleTTS::quit, this);

    Dc::TTSComponent tts(&entity, -1);
    tg = new Dc::EventGui(&entity);
    entity.publish("Start");

    while (runLoop)
    {
      entity.process();
      entity.stepTime();
      Timer_waitDT(10.0*dt);
      RLOG_CPP(1, "Time: " << entity.getTime());
    }

    RLOG(0, "Finishe run loop");
    entity.publish("Stop");
    entity.process();
  }
};

RCS_REGISTER_EXAMPLE(ExampleTTS, "Dexterous Cooperation", "Text to speech");




}   // namespace Dc
