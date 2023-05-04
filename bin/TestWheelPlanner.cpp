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

#include "EntityBase.h"
#include "WheelPlannerComponent.h"
#include "GraphComponent.h"
#include "GraphicsWindow.h"
#include "PhysicsComponent.h"
#include "IKComponent.h"
#include "EventGui.h"
#include "EcsHardwareComponents.h"
#include "WheelStrategy7D.h"
#include "WheelConstraint.h"
#include "WheelNode.h"

#include <PhysicsFactory.h>
#include <ForceDragger.h>
#include <BulletDebugNode.h>
#include <BodyPointDragger.h>
#include <JointWidget.h>
#include <MatNdWidget.h>
#include <ControllerWidgetBase.h>
#include <AStar.h>
#include <IkSolverConstraintRMR.h>
#include <ControllerBase.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_kinematics.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_joint.h>
#include <Rcs_guiFactory.h>
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
static void testPlannerOnly(bool interactive = true)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("+", "Next state in solution");
  Rcs::KeyCatcherBase::registerKey("-", "Previous state in solution");
  Rcs::KeyCatcherBase::registerKey("t", "Run tests");
  Rcs::KeyCatcherBase::registerKey("p", "Plan solution");
  Rcs::KeyCatcherBase::registerKey("P", "Plan solution from to state");

  char xmlFileName[128] = "gWheelPlannerModel.xml";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");
  bool withJointGui = argP.hasArgument("-jointGui", "Joint gui control");
  bool withStateGui = argP.hasArgument("-stateGui", "State gui control");
  bool valgrind = argP.hasArgument("-valgrind", "Without graphics");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  if (interactive == false)
  {
    withJointGui = false;
    withStateGui = false;
    valgrind = true;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);


  //////////////////////////////////////////////////////////////////////
  // Planner
  //////////////////////////////////////////////////////////////////////
  RcsGraph* graph = RcsGraph_create(xmlFileName);
  Dc::WheelStrategy7D strategy(graph);
  if (strategy.test(graph, 100) == false)
  {
    RLOG(0, "Planner test failed");
  }
  strategy.print();
  int solutionIdx = 0;
  std::vector<std::vector<int> > solutionPath;
  solutionPath.push_back(strategy.getState(graph));

  //////////////////////////////////////////////////////////////////////
  // Guis
  //////////////////////////////////////////////////////////////////////
  MatNd* stateMatDes = NULL;
  MatNd* stateMatCurr = NULL;

  if (withStateGui)
  {
    stateMatDes = MatNd_create(Dc::WheelStrategy7D::StateElement::StateMaxIndex, 1);

    std::vector<int> guiState = strategy.getState(graph);
    for (size_t i = 0; i < guiState.size(); ++i)
    {
      MatNd_set(stateMatDes, i, 0, guiState[i]);
    }
    stateMatCurr = MatNd_clone(stateMatDes);

    MatNdWidget* mw = MatNdWidget::create(stateMatDes, stateMatCurr, -32, 32,
                                          "state", &mtx);
    std::vector<std::string> labels;
    labels.push_back("WheelCoord");
    labels.push_back("WheelFlip");
    labels.push_back("WheelRoll");
    labels.push_back("RoboContactRight");
    labels.push_back("RoboContactLeft");
    labels.push_back("HumanContactRight");
    labels.push_back("HumanContactLeft");
    mw->setLabels(labels);
  }

  if (withJointGui)
  {
    if (withStateGui)
    {
      const RcsGraph* constGraph = graph;
      Rcs::JointWidget::create(constGraph, &mtx);
    }
    else
    {
      Rcs::JointWidget::create(graph, &mtx);
    }
  }

  //////////////////////////////////////////////////////////////////////
  // Visualization
  //////////////////////////////////////////////////////////////////////
  char hudText[512] = "";

  Rcs::Viewer* viewer = NULL;
  Rcs::KeyCatcher* kc = NULL;
  Rcs::HUD* hud = NULL;

  if (valgrind == false)
  {
    viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
    hud = new Rcs::HUD();
    viewer->add(hud);
    kc = new Rcs::KeyCatcher();
    viewer->add(kc);
    Rcs::GraphNode* gn = new Rcs::GraphNode(graph);
    viewer->add(gn);
    Rcs::BodyNode* anchor = gn->getBodyNode("WheelPole");
    if (anchor)
    {
      anchor->addChild(new Dc::WheelNode(&strategy));
    }
    viewer->runInThread(&mtx);
  }

  if (interactive == false)
  {
    runLoop = false;
    std::vector<int> from = strategy.getState(graph);
    std::vector<int> to(from.size());

    RLOG(0, "Start planning");
    strategy.setGoal(to);
    double startTime = Timer_getSystemTime();
    solutionPath = Gras::Astar::search(strategy, from);
    bool goalReached = !solutionPath.empty();
    double endTime = Timer_getSystemTime();
    Gras::Astar::printSolution(strategy, solutionPath);
    RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
    RLOG(0, "Time needed: %4.3f", endTime - startTime);
  }



  while (runLoop)
  {
    //////////////////////////////////////////////////////////////////////
    // Forward kinematics
    //////////////////////////////////////////////////////////////////////
    pthread_mutex_lock(&mtx);
    if (withStateGui)
    {
      std::vector<int> guiState;
      for (size_t i = 0; i < Dc::WheelStrategy7D::StateElement::StateMaxIndex; ++i)
      {
        guiState.push_back(lround(MatNd_get(stateMatDes, i, 0)));
      }
      strategy.getQfromState(graph, guiState);
    }

    RcsGraph_setState(graph, NULL, NULL);
    std::vector<int> state = strategy.getState(graph);

    if (stateMatCurr)
    {
      for (size_t i = 0; i < state.size(); ++i)
      {
        MatNd_set(stateMatCurr, i, 0, state[i]);
      }
    }
    pthread_mutex_unlock(&mtx);

    //////////////////////////////////////////////////////////////////////
    // Keycatcher
    //////////////////////////////////////////////////////////////////////
    if (kc && kc->getAndResetKey('q'))
    {
      RMSGS("Quitting run loop");
      runLoop = false;
    }
    else if (kc && kc->getAndResetKey('+'))
    {
      solutionIdx = Math_iClip(solutionIdx + 1, 0, solutionPath.size() - 1);
      RMSGS("Switching to state %d", solutionIdx);
      strategy.getQfromState(graph, solutionPath[solutionIdx]);
    }
    else if (kc && kc->getAndResetKey('-'))
    {
      solutionIdx = Math_iClip(solutionIdx - 1, 0, solutionPath.size() - 1);
      RMSGS("Switching to state %d", solutionIdx);
      strategy.getQfromState(graph, solutionPath[solutionIdx]);
    }
    else if (kc && kc->getAndResetKey('p'))
    {
      RMSGS("Planning new solution:");
      solutionPath.clear();
      solutionIdx = 0;
      std::vector<int> from = strategy.getState(graph);
      std::vector<int> to(from.size());

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);

      if (!goalReached)
      {
        solutionPath.push_back(from);
      }
    }
    else if (kc && kc->getAndResetKey('P'))
    {
      RMSGS("Planning new solution:");
      solutionPath.clear();
      solutionIdx = 0;
      std::vector<int> from(7);
      std::vector<int> to(7);

      RMSGS("Please enter 7 start states:");
      std::cin >> from[0];
      std::cin >> from[1];
      std::cin >> from[2];
      std::cin >> from[3];
      std::cin >> from[4];
      std::cin >> from[5];
      std::cin >> from[6];
      bool startValid = strategy.checkState(from, true);
      RLOG(0, "State is %s", startValid ? "VALID" : "INVALID");

      RMSGS("Please enter 7 goal states:");
      std::cin >> to[0];
      std::cin >> to[1];
      std::cin >> to[2];
      std::cin >> to[3];
      std::cin >> to[4];
      std::cin >> to[5];
      std::cin >> to[6];
      bool goalValid = strategy.checkState(to, true);
      RLOG(0, "State is %s", goalValid ? "VALID" : "INVALID");

      if ((!startValid) || (!goalValid))
      {
        RPAUSE_MSG("Start or goal invalid - hit enter to continue");
      }
      RPAUSE();

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);

      if (!goalReached)
      {
        solutionPath.push_back(from);
      }
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      RMSGS("Running tests");
      bool success = strategy.test(graph, 1000);
      RMSGS("1000 tests %s", success ? "SUCCEEDED" : "FAILED");
    }

    //////////////////////////////////////////////////////////////////////
    // Update head-up display
    //////////////////////////////////////////////////////////////////////
    strcpy(hudText, "State: ");
    char a[64];
    for (size_t i = 0; i < state.size(); ++i)
    {
      snprintf(a, 64, "%d ", state[i]);
      strcat(hudText, a);
    }
    snprintf(a, 64, "\nState is %s ",
             strategy.checkState(state, true) ? "VALID" : "INVALID");
    strcat(hudText, a);
    if (hud)
    {
      hud->setText(hudText);
    }

    Timer_waitDT(0.1);
  }

  if (viewer)
  {
    delete viewer;
  }

  MatNd_destroy(stateMatDes);
  MatNd_destroy(stateMatCurr);
  RcsGraph_destroy(graph);
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testWheelWithoutECS()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("e", "Engage");
  Rcs::KeyCatcherBase::registerKey("W", "Moving to goal state");
  Rcs::KeyCatcherBase::registerKey("m", "Initialize movement");
  Rcs::KeyCatcherBase::registerKey("o", "Moving to gui state");
  Rcs::KeyCatcherBase::registerKey("p", "Planning to gui state");
  Rcs::KeyCatcherBase::registerKey(" ", "Toggle pause modus");
  Rcs::KeyCatcherBase::registerKey("t", "Run tests");
  Rcs::KeyCatcherBase::registerKey("f", "Toggle time freeze");
  Rcs::KeyCatcherBase::registerKey("k", "Toggle speed-up");
  Rcs::KeyCatcherBase::registerKey("a", "Plan to state [0 0]");
  Rcs::KeyCatcherBase::registerKey("b", "Plan to state [7 -3] (home)");
  Rcs::KeyCatcherBase::registerKey("A", "Plan to state [0 -6]");
  Rcs::KeyCatcherBase::registerKey("z", "Print information to console");

  runLoop = true;
  char xmlFileName[128] = "cSimpleWheel.xml";
  double alpha = 0.05, horizon = 1.0, dt = 0.025, ttc = 5.0;
  unsigned int speedUp = 1, loopCount = 0, speedLimitViolations = 0;
  bool enableSpeedup = true;

  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName);
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                   horizon);
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %g)", alpha);
  argP.getArgument("-dt", &dt, "Sampling time for controller (default is %f)",
                   dt);
  argP.getArgument("-ttc", &ttc, "Transition time (default is %f)", ttc);
  argP.getArgument("-speedUp", &speedUp, "Speed-up factor (default is %d)",
                   speedUp);
  bool valgrind = argP.hasArgument("-valgrind", "Without graphics");
  bool zigzag = argP.hasArgument("-zigzag", "Zig-zag trajectory");
  bool withStateGui = argP.hasArgument("-stateGui", "State gui control");
  bool withTaskGui = argP.hasArgument("-taskGui", "Task space gui (passive)");
  bool withJointGui = argP.hasArgument("-jointGui", "Joint gui (passive)");
  bool withActiveJointGui = argP.hasArgument("-activeJointGui", "Joint gui (active)");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                         "shadows and anti-aliasing");
  bool pause = argP.hasArgument("-pause", "Hit enter for each iteration");
  bool plot = argP.hasArgument("-plot", "Plot trajectory");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool noLimits = argP.hasArgument("-noLimits", "Ignore joint, speed and "
                                   "collision limits");
  bool testCopy = argP.hasArgument("-copy", "Test copying TrajectoryController");

  if (noLimits)
  {
    noSpeedCheck = true;
    noJointCheck = true;
    noCollCheck = true;
  }

  if (argP.hasArgument("-h"))
  {
    return;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  //////////////////////////////////////////////////////////////////////
  // Trajectory and IK
  //////////////////////////////////////////////////////////////////////
  tropic::TrajectoryControllerBase* tc = NULL;

  if (zigzag)
  {
    tc = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(xmlFileName, horizon);
  }
  else
  {
    tc = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(xmlFileName, horizon);
  }

  ControllerBase* controller = tc->getInternalController();
  RCHECK(controller);

  const unsigned int nx = controller->getTaskDim();
  const unsigned int nTasks = controller->getNumberOfTasks();
  const unsigned int dof = controller->getGraph()->dof;
  MatNd* a_des     = MatNd_create(nTasks, 1);
  MatNd* x_des     = MatNd_create(nx, 1);
  MatNd* x_curr    = MatNd_create(nx, 1);
  MatNd* dx_des    = MatNd_create(nx, 1);
  MatNd* dq_des    = MatNd_create(dof, 1);
  MatNd* q_dot_des = MatNd_create(dof, 1);
  MatNd* dH        = MatNd_create(1, dof);
  MatNd* dH_ca     = MatNd_create(1, dof);
  MatNd* q_gui     = MatNd_clone(controller->getGraph()->q);
  controller->computeX(x_des);
  MatNd_copy(x_curr, x_des);
  double dt_step = dt;

  bool success = controller->readActivationsFromXML(a_des);
  RCHECK(success);

  for (unsigned int i=0; i<a_des->m; ++i)
  {
    tc->setActivation(i, (a_des->ele[i]>0.0) ? true : false);
  }


  if (testCopy)
  {
    tropic::TrajectoryControllerBase* tcCopy = new tropic::TrajectoryControllerBase(*tc);
    delete tc;
    tc = tcCopy;
    controller = tc->getInternalController();
    RCHECK(controller);
    tc->print();
  }

  IkSolverConstraintRMR ikSolver(controller);

  Dc::WheelStrategy7D strategy(controller->getGraph());
  std::vector<int> state0 = strategy.getState(controller->getGraph());
  std::vector<int> state1(state0.size());
  std::vector<std::vector<int>> solutionPath;
  Dc::WheelConstraint wConstraint(tc, &strategy);

  //////////////////////////////////////////////////////////////////////
  // Visualization
  //////////////////////////////////////////////////////////////////////
  char hudText[512] = "";
  double time = 0.0;

  Rcs::Viewer* viewer = NULL;
  Rcs::KeyCatcher* kc = NULL;
  Rcs::HUD* hud = NULL;
  Rcs::BodyPointDragger* dragger = NULL;

  if (valgrind==false)
  {
    viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
    viewer->setCameraTransform(-3.4, 4.5, 2.5, 0.3, 0.1, -1.1);
    hud = new Rcs::HUD();
    viewer->add(hud);
    kc = new Rcs::KeyCatcher();
    viewer->add(kc);
    Rcs::GraphNode* gn = new Rcs::GraphNode(controller->getGraph());
    viewer->add(gn);
    if (controller->getCollisionMdl() != NULL)
    {
      Rcs::VertexArrayNode* cn =
        new Rcs::VertexArrayNode(controller->getCollisionMdl()->cp,
                                 osg::PrimitiveSet::LINES, "RED");
      cn->toggle();
      viewer->add(cn);
    }
    dragger = new Rcs::BodyPointDragger();
    dragger->scaleDragForce(0.05);
    viewer->add(dragger);
    Rcs::BodyNode* anchor = gn->getBodyNode("WheelPole");
    RCHECK(anchor);
    anchor->addChild(new Dc::WheelNode(&strategy));
    viewer->runInThread(&mtx);
  }
  else
  {
    RMSGS("Initializing movement");
    auto moveSet = wConstraint.createInitMove(ttc);
    tc->add(moveSet);
    moveSet->apply(tc->getTrajectoriesRef());
  }

  //////////////////////////////////////////////////////////////////////
  // Guis
  //////////////////////////////////////////////////////////////////////
  MatNd* stateMatDes = NULL;
  MatNd* stateMatCurr = NULL;

  if (withStateGui)
  {
    stateMatDes = MatNd_create(Dc::WheelStrategy7D::StateElement::StateMaxIndex, 1);

    std::vector<int> guiState = strategy.getState(controller->getGraph());
    for (size_t i=0; i<guiState.size(); ++i)
    {
      MatNd_set(stateMatDes, i, 0, guiState[i]);
    }
    stateMatCurr = MatNd_clone(stateMatDes);

    MatNdWidget* mw = MatNdWidget::create(stateMatDes, stateMatCurr, -40, 40,
                                          "state", &mtx);
    std::vector<std::string> labels;
    labels.push_back("WheelCoord");
    labels.push_back("WheelFlip");
    labels.push_back("WheelRoll");
    labels.push_back("RoboContactRight");
    labels.push_back("RoboContactLeft");
    labels.push_back("HumanContactRight");
    labels.push_back("HumanContactLeft");
    mw->setLabels(labels);
  }

  if (withTaskGui)
  {
    // We launch it in "show only" mode, so that there are no modifications
    // to the activations. Our interface is not very well designed, so that
    // we have to do some casting here.
    ControllerWidgetBase::create(controller, a_des, x_des, x_curr, &mtx, true);
  }

  if (withJointGui)
  {
    Rcs::JointWidget::create((const RcsGraph*) controller->getGraph(), &mtx);
  }

  if (withActiveJointGui)
  {
    Rcs::JointWidget::create(controller->getGraph(), &mtx, q_gui, controller->getGraph()->q);
  }

  Rcs::ViaPointSequencePlotter* plotter = NULL;
  const Rcs::ViaPointSequence* viaSeq = NULL;

  if (plot)
  {
    tropic::ViaPointTrajectory1D* viaTraj =
      dynamic_cast<tropic::ViaPointTrajectory1D*>(tc->getTrajectory("RobotLeft")->getTrajectory1D(2));
    RCHECK(viaTraj);
    viaSeq = viaTraj->getViaSequence();
    RCHECK(viaSeq);
    viaTraj->getViaSequence()->setTurboMode(false);
    plotter = new Rcs::ViaPointSequencePlotter();
  }

  while (runLoop)
  {

    pthread_mutex_lock(&mtx);

    /////////////////////////////////////////////////////////////////
    // Trajectory
    /////////////////////////////////////////////////////////////////
    double dt_traj = Timer_getSystemTime();
    double t_end = tc->step(dt_step);
    dt_traj = Timer_getSystemTime() - dt_traj;
    tc->getPosition(0.0, x_des);
    tc->getActivation(a_des);

    /////////////////////////////////////////////////////////////////
    // Inverse kinematics
    /////////////////////////////////////////////////////////////////
    if (!withActiveJointGui)
    {
      controller->computeCollisionModel();
      controller->getCollisionGradient(dH_ca);
      RcsGraph_limitJointSpeeds(controller->getGraph(), dH_ca,
                                dt, RcsStateIK);

      controller->computeDX(dx_des, x_des);
      controller->computeJointlimitGradient(dH);
      MatNd_addSelf(dH, dH_ca);
      MatNd_constMulSelf(dH, alpha);

      if (valgrind == false)
      {
        dragger->addJointTorque(dH, controller->getGraph());
      }

      ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, 0.0);

      double scale = RcsGraph_limitJointSpeeds(controller->getGraph(), dq_des,
                                               dt, RcsStateFull);
      if (scale<1.0)
      {
        RLOG(4, "Scaling down joint speeds by factor %f", scale);
        MatNd_constMulSelf(dq_des, 0.99999);
        speedLimitViolations++;
      }


      MatNd_addSelf(controller->getGraph()->q, dq_des);
      MatNd_constMul(q_dot_des, dq_des, 1.0/dt);
    }

    if (withActiveJointGui)
    {
      MatNd_copy(controller->getGraph()->q, q_gui);
    }

    RcsGraph_setState(controller->getGraph(), NULL, q_dot_des);
    bool poseOK = controller->checkLimits(!noJointCheck, !noCollCheck,
                                          !noSpeedCheck);

    /////////////////////////////////////////////////////////////////
    // Compute current search state
    /////////////////////////////////////////////////////////////////
    std::vector<int> state = strategy.getState(controller->getGraph());
    if (stateMatCurr)
    {
      for (unsigned int i = 0; i < state.size(); ++i)
      {
        stateMatCurr->ele[i] = state[i];
      }
    }

    if (plotter)
    {
      plotter->plot2(*viaSeq, -0.2, 5.2, dt, 1);
    }


    pthread_mutex_unlock(&mtx);

    //////////////////////////////////////////////////////////////////////
    // Keycatcher
    //////////////////////////////////////////////////////////////////////
    if (kc && kc->getAndResetKey('q'))
    {
      RMSGS("Quitting run loop");
      runLoop = false;
    }
    else if (kc && kc->getAndResetKey('z'))
    {
      RcsCollisionModel_fprintCollisions(stdout, controller->getCollisionMdl(),
                                         1000.0);
      RcsGraph_fprintModelState(stdout, controller->getGraph(),
                                controller->getGraph()->q, NULL, 0);
    }
    else if (kc && kc->getAndResetKey('e'))
    {
      RMSGS("Engage");
      auto moveSet = wConstraint.createEngageMove(ttc, 0.0);
      moveSet->add(wConstraint.createFingersOpenMove(ttc));
      moveSet->add(wConstraint.createFingersClosedMove(1.5*ttc));
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('f'))
    {
      dt_step = (dt_step == 0.0) ? dt : 0.0;
      RMSGS("%s time", (dt_step == 0.0) ? "FREEZING" : "UNFREEZING");
    }
    else if (kc && kc->getAndResetKey('W'))
    {
      RMSGS("Moving to state");
      auto moveSet = wConstraint.moveTo(state1, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('m'))
    {
      RMSGS("Initializing movement");
      auto moveSet = wConstraint.createInitMove(ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
      REXEC(4)
      {
        RcsGraph_fprintModelState(stdout, controller->getGraph(),
                                  controller->getGraph()->q, NULL, 0);
      }
    }
    else if (kc && kc->getAndResetKey('o'))
    {
      std::vector<int> goalState(Dc::WheelStrategy7D::StateMaxIndex);

      if (withStateGui)
      {
        RMSGS("Moving to state from GUI ");
        for (int i=0; i< Dc::WheelStrategy7D::StateElement::StateMaxIndex; ++i)
        {
          int state_i = lround(MatNd_get(stateMatDes, i, 0));
          goalState[i] = state_i;
          fprintf(stderr, "%d ", state_i);
        }
      }
      else
      {
        RMSGS("Please enter 5 states:");
        std::cin >> goalState[0];
        std::cin >> goalState[1];
        std::cin >> goalState[2];
        std::cin >> goalState[3];
        std::cin >> goalState[4];
      }

      auto moveSet = wConstraint.moveTo(goalState, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('p'))
    {
      RMSGS("Planning new solution:");
      solutionPath.clear();
      std::vector<int> from = strategy.getState(controller->getGraph());
      std::vector<int> to(from.size());

      if (stateMatDes != NULL)
      {
        for (size_t i = 0; i<to.size(); ++i)
        {
          int state_i = lround(MatNd_get(stateMatDes, i, 0));
          to[i] = state_i;
        }
      }

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);


      auto moveSet = wConstraint.addSolution(solutionPath, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey(' '))
    {
      pause = !pause;
      RMSGS("Pause mode is %s", pause ? "ON" : "OFF");
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      RMSGS("Running tests");
      bool success = strategy.test(controller->getGraph(), 1000);
      RMSGS("1000 tests %s", success ? "SUCCEEDED" : "FAILED");
    }
    else if (kc && kc->getAndResetKey('a'))
    {
      RLOG(0, "Plan and perform sequence to [0 0]");
      solutionPath.clear();
      std::vector<int> from = strategy.getState(controller->getGraph());
      std::vector<int> to { 0, 0, 0, 0, 0, 0, 0 };

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);

      auto moveSet = wConstraint.addSolution(solutionPath, ttc);

      for (size_t i=0; i<moveSet->numSets(false); ++i)
      {
        if (moveSet->getSet(i)->getClassName()=="StateChange")
        {
          RLOG_CPP(0, "Set " << i << " : " << moveSet->getSet(i)->getEndTime());
        }
      }

      if (testCopy)
      {
        auto testSet = moveSet;
        RLOG(0, "Sets are %s",
             *(moveSet.get())==*(testSet.get()) ? "EQUAL" : "DIFFERENT");
        RLOG(0, "Sets are %s",
             moveSet->isEqual(testSet) ? "EQUAL" : "DIFFERENT");
        RLOG_CPP(0, "constraints: " << moveSet->numConstraints() << " "
                 << testSet->numConstraints() << " sets: " << moveSet->numSets()
                 << " " << testSet->numSets());
      }

      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('A'))
    {
      RLOG(0, "Plan and perform sequence to [0 -6]");
      solutionPath.clear();
      std::vector<int> from = strategy.getState(controller->getGraph());
      std::vector<int> to { 0, -6, 0, 0, 0, 0, 0 };

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);


      auto moveSet = wConstraint.addSolution(solutionPath, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('b'))
    {
      RLOG(0, "Plan and perform sequence to [11 -3]");
      solutionPath.clear();
      std::vector<int> from = strategy.getState(controller->getGraph());
      std::vector<int> to { 11, -3, 0, 0, 0, 0, 0 };

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);

      auto moveSet = wConstraint.addSolution(solutionPath, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('B'))
    {
      RLOG(0, "Plan and perform sequence to [11 3]");
      solutionPath.clear();
      std::vector<int> from = strategy.getState(controller->getGraph());
      std::vector<int> to { 11, 3, 0, 0, 0, 0, 0 };

      RLOG(0, "Start planning");
      strategy.setGoal(to);
      double startTime = Timer_getSystemTime();
      solutionPath = Gras::Astar::search(strategy, from);
      bool goalReached = !solutionPath.empty();
      double endTime = Timer_getSystemTime();
      Gras::Astar::printSolution(strategy, solutionPath);
      RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
      RLOG(0, "Time needed: %4.3f", endTime - startTime);

      auto moveSet = wConstraint.addSolution(solutionPath, ttc);
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('k'))
    {
      enableSpeedup = !enableSpeedup;
      RMSGS("Speed-up mode is %s", enableSpeedup ? "ON" : "OFF");
    }

    //////////////////////////////////////////////////////////////////////
    // Update head-up display
    //////////////////////////////////////////////////////////////////////
    bool stateValid = strategy.checkState(state, true);
    sprintf(hudText, "Time: %.3f   End time: %.3f\nState: ", time, t_end);
    char a[64];
    for (size_t i = 0; i<state.size(); ++i)
    {
      snprintf(a, 64, "%d ", state[i]);
      strcat(hudText, a);
    }
    snprintf(a, 64, " (%s)", stateValid ? "VALID" : "INVALID");
    strcat(hudText, a);
    snprintf(a, 64, "\nRobot pose %s", poseOK ? "VALID" : "VIOLATES LIMITS");
    strcat(hudText, a);
    snprintf(a, 64, "\nSpeed-up is %d", enableSpeedup ? speedUp : 1);
    strcat(hudText, a);
    snprintf(a, 64, "\nSpeed-limit violations: %d", speedLimitViolations);
    strcat(hudText, a);

    if (hud)
    {
      hud->setText(hudText);
    }

    time += dt_step;
    loopCount++;
    if (loopCount%speedUp==0 || enableSpeedup==false)
    {
      Timer_waitDT(dt);
    }

    if (pause)
    {
      RPAUSE();
    }

    if (!poseOK)
    {
      RPAUSE_DL(1);
    }

    if (valgrind)
    {
      REXEC(1)
      {
        std::cout << hudText << std::endl;
      }

      if (tc->getTimeOfLastGoal()<=0.0)
      {
        runLoop = false;
      }
    }
  }


  if (viewer)
  {
    delete viewer;
  }

  delete tc;

  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(dx_des);
  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(dH);
  MatNd_destroy(dH_ca);
  MatNd_destroy(stateMatDes);
  MatNd_destroy(stateMatCurr);
  MatNd_destroy(q_gui);

  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger : public ForceDragger
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
static void testWheelWithECS(int argc, char** argv)
{
  CmdLineParser argP;
  double dt = 0.01, ttc = 8.0, alpha = 0.02;
  unsigned int speedUp = 1, loopCount = 0;
  size_t queueSize = 0;
  char cfgFile[64] = "cSimpleWheel.xml";
  char physicsEngine[32] = "Bullet";
  char physicsCfg[128] = "config/physics/physics.xml";
  argP.getArgument("-f", cfgFile, "Configuration file name "
                   "(default is %s)", cfgFile);
  argP.getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)",
                   speedUp);
  argP.getArgument("-physicsEngine", physicsEngine,
                   "Physics engine (default: \"%s\")", physicsEngine);
  argP.getArgument("-physics_config", physicsCfg, "Configuration file name"
                   " for physics (default: %s)", physicsCfg);
  argP.getArgument("-dt", &dt, "Time step (default is %f)", dt);
  argP.getArgument("-ttc", &ttc, "Transition time (default: %f)", ttc);
  argP.getArgument("-alpha", &alpha, "Null space scaler (default: %f)", alpha);
  bool noEventGui = argP.hasArgument("-noEventGui", "Don't launch EventGui");
  bool noSpeedCheck = argP.hasArgument("-nospeed", "No speed limit checks");
  bool noJointCheck = argP.hasArgument("-nojl", "Disable joint limit checks");
  bool noCollCheck = argP.hasArgument("-nocoll", "Disable collision checks");
  bool seqSim = argP.hasArgument("-sequentialPhysics", "Physics simulation "
                                 "step in updateGraph()");
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

  Dc::EntityBase entity;
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
  auto setComplianceCommand = entity.registerEvent<std::vector<double>>("SetComplianceCommand");

  entity.registerEvent<>("Start");
  entity.registerEvent<>("Stop");
  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &quit);

  Dc::GraphicsWindow viewer(&entity, startViewerWithStartEvent, seqViewer, simpleGraphics);
  Dc::GraphComponent graphC(&entity, graph);
  Dc::WheelPlannerComponent dmpc(&entity, controller, !zigzag);
  Dc::IKComponent ikc(&entity, controller);
  ikc.setSpeedLimitCheck(!noSpeedCheck);
  ikc.setJointLimitCheck(!noJointCheck);
  ikc.setCollisionCheck(!noCollCheck);
  ikc.setAlpha(alpha);

  std::vector<Dc::ComponentBase*> hwc = Dc::getHardwareComponents(entity, graph);

  if (hwc.empty())
  {

    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      JNT->ctrlType = RCSJOINT_CTRL_POSITION;
    }

    Dc::PhysicsComponent* pc = new Dc::PhysicsComponent(&entity, graph, physicsEngine,
                                                        physicsCfg, !seqSim);
    if (pc->getPhysicsSimulation())
    {
      osg::ref_ptr<NamedBodyForceDragger> dragger =
        new NamedBodyForceDragger(pc->getPhysicsSimulation());

      viewer.add(dragger.get());
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


  delete controller;

  viewer.setKeyCallback('d', [&entity](char k)
  {
    RLOG(0, "Reseting IME failure flag");
    entity.publish("ResetErrorFlag");
  }, "Reseting IME failure flag");

  viewer.setKeyCallback('m', [&entity, ttc](char k)
  {
    RLOG(0, "Move robot to initial pose");
    entity.publish("InitializeMovement", ttc);
  }, "Move robot to initial pose");

  viewer.setKeyCallback('a', [&entity](char k)
  {
    RLOG(0, "Plan and perform sequence to [0 0]");
    entity.publish("PlanToFlipAndExecute", 0, 0);
  }, "Plan and perform sequence to [0 0]");

  viewer.setKeyCallback('A', [&entity](char k)
  {
    RLOG(0, "Plan and perform sequence to [0 -6]");
    entity.publish("PlanToFlipAndExecute", 0, -6);
  }, "Plan and perform sequence to [0 -6]");

  viewer.setKeyCallback('b', [&entity](char k)
  {
    RLOG(0, "Plan and perform sequence to [11 -3]");
    entity.publish("PlanToFlipAndExecute", 11, -3);
  }, "Plan and perform sequence to [11 -3]");

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
    Dc::EventGui::create(&entity);
  }

  // Start threads (if any)
  RPAUSE_MSG_DL(1, "Start");
  entity.publish("Start");
  int nIter = entity.processUntilEmpty(10);
  RLOG(1, "Start took %d process() calls, queue is %zu",
       nIter, entity.queueSize());

  // Initialization sequence
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

  Rcs::GraphNode* gn = viewer.getGraphNodeById("Physics");
  if (gn)
  {
    Rcs::BodyNode* anchor = gn->getBodyNode("WheelPole");
    if (anchor)
    {
      anchor->addChild(new Dc::WheelNode(dmpc.getStrategy()));
      entity.publish("Render");
      entity.processUntilEmpty(10);
    }
  }

  RPAUSE_MSG_DL(1, "EnableCommands");
  entity.publish("EnableCommands");
  nIter = entity.processUntilEmpty(10);
  RLOG(1, "InitFromState++ took %d process() calls, queue is %zu",
       nIter, entity.queueSize());
  RPAUSE_MSG_DL(1, "Enter runLoop");


  while (runLoop)
  {
    double dtProcess = Timer_getSystemTime();
    updateGraph->call(graphC.getGraph());

    // This updates the TaskGui and filters the commands
    computeKinematics->call(graphC.getGraph());

    postUpdateGraph->call(ikc.getGraph(), graphC.getGraph());

    // This updates the TaskGui and filters the commands
    computeTrajectory->call(ikc.getGraph());

    // This computes the IK with the planner's inputs into the IKComponents
    // internal q_des:  a, x -> q
    setTaskCommand->call(dmpc.getActivationPtr(), dmpc.getTaskCommandPtr());

    // This is called right before sending the commands
    // checkEmergencyStop->call();

    // Distribute the IKComponents internal command to all motor components
    setJointCommand->call(ikc.getJointCommandPtr());   // motors <- q

    // Compute the compliance wrench depending on the task space velocities
    // of the hand-wheel relative tasks.
    auto wrench = dmpc.getComplianceWrench();
    setComplianceCommand->call(wrench);

    // This triggers graphics updates in some components
    setRenderCommand->call();

    queueSize = std::max(entity.queueSize(), queueSize);

    entity.process();
    entity.stepTime();

    dtProcess = Timer_getSystemTime() - dtProcess;


    char text[256];
    snprintf(text, 256, "Time: %.3f   dt: %.1f msec (< %.1f msec)   queue: %zu",
             entity.getTime(), 1.0e3*dtProcess, 1.0e3*entity.getDt(), queueSize);
    entity.publish("SetTextLine", std::string(text), 1);

    snprintf(text, 256, "Motion time: %5.3f\nState: ",
             dmpc.getMotionEndTime());
    std::vector<int> searchState = dmpc.getState();
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
    if (loopCount%speedUp == 0)
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
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  char directory[128] = "config/xml/DexterousCooperation/WheelTurning";
  int mode = 0;
  CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is %s)", directory);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(directory);

  switch (mode)
  {
    case 1:
      testPlannerOnly();
      break;

    case 2:
      testWheelWithoutECS();
      break;

    case 3:
      testWheelWithECS(argc, argv);
      break;

    default:
      RMSG("No mode %d", mode);
  }



  if (argP.hasArgument("-h") || (mode==0))
  {
    Rcs_printResourcePath();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
    printf("Mode 1: Planner test only (with graphics)\n");
    printf("Mode 2: Full interactive test without event system\n");
    printf("Mode 3: Full interactive test with event system\n\n");
  }

  RcsGuiFactory_shutdown();
  xmlCleanupParser();

  RLOG(0, "Thanks for using the TestWheelPlanner\n");

  return 0;
}
