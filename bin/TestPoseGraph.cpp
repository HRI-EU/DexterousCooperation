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

#include <PoseGraph.h>
#include <DynamicPoseGraph.h>
#include <ConstraintFactory.h>
#include <RcsViewer.h>
#include <GraphNode.h>
#include <ControllerWidgetBase.h>
#include <KeyCatcher.h>

#include <ConstraintFactory.h>
#include <ControllerBase.h>
#include <IkSolverRMR.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_guiFactory.h>
#include <SegFaultHandler.h>

#include <csignal>

RCS_INSTALL_ERRORHANDLERS

using namespace Rcs;

bool runLoop = true;


/*******************************************************************************
 *
 ******************************************************************************/
void quit(int)
{
  RLOG(0, "Quit::quit()");
  runLoop = false;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testPoseGraphCreation()
{
  CmdLineParser argP;
  double lambda = 0.0, alpha = 0.05;
  std::string xmlFileName = "cAction.xml";
  std::string directory = "config/xml/DexterousCooperation/PoseGraph/mode1";
  argP.getArgument("-f", &xmlFileName, "Configuration file name "
                   "(default is %s)", xmlFileName.c_str());
  argP.getArgument("-dir", &directory, "Configuration file directory "
                   "(default is %s)", directory.c_str());
  bool valgrind = argP.hasArgument("-valgrind", "Start without UIs");
  bool withGui = argP.hasArgument("-gui", "Start without controller GUI");
  Rcs_addResourcePath(directory.c_str());

  if (argP.hasArgument("-h"))
  {
    return;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  double t_calc = Timer_getSystemTime();

  DynamicPoseGraph poseGraph(xmlFileName);
  auto tSet = tropic::ConstraintFactory::create(directory + "/trajectory.xml");
  RLOG(1, "DynamicPoseGraph end time: %f", tSet->getEndTime());

  poseGraph.addTrajectory(tSet);

  std::vector<double> times;
  times.push_back(5);
  times.push_back(11);
  times.push_back(23);
  times.push_back(29);
  times.push_back(35);
  times.push_back(53);
  times.push_back(59);

  ControllerBase* seq = poseGraph.createLinks(times);
  t_calc = Timer_getSystemTime() - t_calc;

  int nErrors = 0;
  RcsGraph_check(seq->getGraph(), &nErrors, NULL);
  RLOG(0, "Initialization took %.2f msec", 1.0e3*t_calc);
  RLOG(0, "%s - found %d errors in graph", (nErrors==0?"SUCCESS":"FAILURE"),
       nErrors);


  REXEC(3)
  {
    RcsGraph_writeDotFile(seq->getGraph(), "poseGraph.dot");
    FILE* outFile = fopen("poseGraph.xml", "w+");
    RCHECK(outFile);
    RcsGraph_fprintXML(outFile, seq->getGraph());
    fclose(outFile);
  }

  RLOG(0, "End time: %f", poseGraph.tc->getRootSet().getEndTime());

  MatNd* a_des = MatNd_create(seq->getNumberOfTasks(), 1);
  MatNd_setElementsTo(a_des, 1.0);
  MatNd* x_des = MatNd_create(seq->getTaskDim(), 1);
  seq->computeX(x_des);
  MatNd* x_curr = MatNd_clone(x_des);
  MatNd* dx_des = MatNd_create(seq->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, seq->getGraph()->nJ);
  MatNd* dq_des = MatNd_create(seq->getGraph()->dof, 1);
  Rcs::IkSolverRMR ikSolver(seq);

  Rcs::Viewer viewer;
  osg::ref_ptr<Rcs::KeyCatcher> kc;

  if (!valgrind)
  {
    viewer.add(new Rcs::GraphNode(seq->getGraph()));
    kc = new Rcs::KeyCatcher();
    viewer.add(kc.get());
    viewer.runInThread(&mtx);

    if (withGui)
    {
      ControllerWidgetBase::create(seq, a_des, x_des, x_curr, &mtx);
    }
  }
  else
  {
    runLoop = false;
  }

  if (!valgrind)
  {
    RPAUSE_DL(1);
  }

  while (runLoop)
  {
    pthread_mutex_lock(&mtx);
    seq->computeDX(dx_des, x_des);
    seq->computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha);
    ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
    MatNd_addSelf(seq->getGraph()->q, dq_des);
    RcsGraph_setState(seq->getGraph(), NULL, NULL);
    pthread_mutex_unlock(&mtx);

    if (kc.valid() && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }


    Timer_waitDT(0.01);
  }

  RMSG("To play around with the model, just type:\nbin/Rcs -m 5 "
       "-f cSequencePolar.xml -algo 1 -lambda 0 -setDefaultStateFromInit "
       "-alpha 1 ");

  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dq_des);
  delete seq;
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
//#define PREDICT_DEBUG
MatNd* createPoseSequence(const ControllerBase* controller_,
                          const std::vector<double>& times,
                          std::shared_ptr<tropic::ConstraintSet> tSet)
{
  const double dt = 0.05, alpha = 0.05, lambda = 1.0e-8;
  ControllerBase* controller = new ControllerBase(*controller_);
  IkSolverRMR ikSolver(controller);
  tropic::ViaTrajectoryController tc(controller, 1.0);
  tc.takeControllerOwnership(true);
  tc.setTurboMode(false);
  bool success = tc.addAndApply(tSet, true);

  if (!success)
  {
    RLOG(1, "Failed to add trajectory constraint");
    return NULL;
  }
  else
  {
    RLOG(1, "Trajectory is %f sec long", tc.getRootSet().getEndTime());
  }

  // Compute the trajectory polynomial parameters until the end of the motion
  tc.step(0.0);

#if defined (PREDICT_DEBUG)
  Rcs::Viewer viewer;
  viewer.add(new Rcs::GraphNode(controller->getGraph()));
  viewer.runInThread();
  RPAUSE_DL(0);
#endif

  MatNd* a_des = MatNd_create(controller->getNumberOfTasks(), 1);
  controller->readActivationsFromXML(a_des);
  MatNd* x_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller->getGraph()->nJ);
  MatNd* dq_des = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* postures = MatNd_create(times.size(), controller->getGraph()->dof);
  postures->m = 0;

  // We need to check the current state, otherwise we don't have any
  // information about it.
  MatNd q_row = MatNd_fromPtr(1, controller->getGraph()->dof,
                              controller->getGraph()->q->ele);

  double ti = 0.0;

  for (size_t i=0; i<times.size(); ++i)
  {

    while (fabs(ti-times[i])>1.0e-8)
    {
      double dt_step = std::min(dt, times[i]-ti);
      ti += dt_step;
      tc.getPosition(ti, x_des);
      controller->computeDX(dx_des, x_des);
      controller->computeJointlimitGradient(dH);
      MatNd_constMulSelf(dH, alpha);
      ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
      MatNd_addSelf(controller->getGraph()->q, dq_des);
      RcsGraph_setState(controller->getGraph(), NULL, NULL);
    }

    MatNd_appendRows(postures, &q_row);

#if defined (PREDICT_DEBUG)
    RLOG(0, "t = %.3f   row=%u   check is %s",
         ti, postures->m, controller->checkLimits() ? "OK" : "BAD");
    RPAUSE_DL(0);
#endif
  }

  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dq_des);

  return postures;
}

static void testGenericPoseGraphCreation()
{
  CmdLineParser argP;
  std::string directory = "config/xml/DexterousCooperation/PoseGraph";
  std::string example = "ex0";
  argP.getArgument("-dir", &directory, "Configuration file directory "
                   "(default is %s)", directory.c_str());
  argP.getArgument("-example", &example, "Example number (default is %s)",
                   example.c_str());

  directory += "/";
  directory += example;

  Rcs_addResourcePath(directory.c_str());

  if (argP.hasArgument("-h"))
  {
    RLOG(0, "Run:\n\nbin/TestPoseGraph -m 2 "
         "-example <ex0, ex1, ex2, ex3, ex4>\n\n");
    return;
  }

  PoseGraph poseGraph;
  std::vector<PoseGraph::Adjacency> adja;
  MatNd* postures = NULL;
  ControllerBase* seq = NULL;

  if (example=="ex0")
  {
    ControllerBase controller("cExample0.xml");

    std::vector<double> times{5, 11, 23, 29, 35, 53, 59};
    auto ts = tropic::ConstraintFactory::create(directory + "/trajectory.xml");
    RLOG(1, "DynamicPoseGraph end time: %f", ts->getEndTime());
    postures = createPoseSequence(&controller, times, ts);

    PoseGraph::Adjacency a;

    a.bdyName = "Box_v";
    a.adjacencyList = {-1, 0, -1, 2, 3, -1, 5};
    a.originalTask = "Phi_Box";
    a.relaxedTask = "Polar_Box";
    adja.push_back(a);

    a.bdyName = "PowerGrasp_R";
    a.adjacencyList = {-1, 0, 1, -1, 3, 4, -1};
    a.originalTask = "ABC_R";
    a.relaxedTask = "Polar_R";
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "PowerGrasp_L";
    a.adjacencyList = {-1, -1, 1, 2, -1, 4, 5};
    a.originalTask = "ABC_L";
    a.relaxedTask = "Polar_L";
    a.fixLastPose = false;
    adja.push_back(a);

    double offset[3];
    Vec3d_set(offset, -2.5, 0.0, 0.0);

    seq = poseGraph.create(&controller, adja, postures, offset);
  }
  else if (example=="ex1")
  {
    RLOG(0, "Creating box surface sliding problem");
    ControllerBase controller("cExample1.xml");
    PoseGraph::Adjacency a;

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1};
    a.originalTask = "Object";
    a.relaxedTask = "ObjectYZ";
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3};
    a.originalTask = "Hand Pose";
    a.relaxedTask = "Hand Dist";
    a.fixLastPose = false;
    adja.push_back(a);

    double offset[3];
    Vec3d_set(offset, 0.0, -2.5, 0.0);

    seq = poseGraph.create(&controller, adja, postures, offset);
  }
  else if (example=="ex2")
  {
    RLOG(0, "Creating light traverse sliding problem");
    ControllerBase controller("cExample2.xml");
    PoseGraph::Adjacency a;

    a.bdyName = "Traverse_1";
    a.adjacencyList = {-1, 0, -1, 2, -1};
    a.originalTask = "Traverse";
    a.relaxedTask = "";
    a.fixLastPose = true;
    adja.push_back(a);

    a.bdyName = "PowerGrasp_R_1";
    a.adjacencyList = {-1, -1, 1, -1, 3};
    a.originalTask = "HandPose6d";
    a.relaxedTask = "HandPose4d";
    a.fixLastPose = false;
    adja.push_back(a);

    double offset[3];
    Vec3d_set(offset, 0.0, 1.5, 0.0);

    seq = poseGraph.create(&controller, adja, postures, offset);
  }
  else if (example=="ex3")
  {
    RLOG(0, "Creating box pole sliding problem");
    ControllerBase controller("cExample3.xml");
    PoseGraph::Adjacency a;

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1};
    a.originalTask = "Box";
    a.relaxedTask = "";
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3};
    a.originalTask = "Hand-Box pose";
    a.relaxedTask = "Hand-Box distance";
    a.fixLastPose = false;
    adja.push_back(a);

    double offset[3];
    Vec3d_set(offset, 0.0, -1.5, 0.0);

    seq = poseGraph.create(&controller, adja, postures, offset);
  }
  else if (example=="ex4")
  {
    RLOG(0, "Creating box pole sliding problem");
    ControllerBase controller("cExample4.xml");
    PoseGraph::Adjacency a;

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1};
    // a.adjacencyList = {-1, 0, -1};
    a.originalTask = "Box";
    a.relaxedTask = "";
    a.fixFirstPose = false;
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3};
    // a.adjacencyList = {-1, -1, 1};
    a.originalTask = "Hand-Box pose";
    a.relaxedTask = "Hand-Box distance";
    a.fixLastPose = false;
    adja.push_back(a);

    double offset[3];
    Vec3d_set(offset, 0.0, -1.5, 0.0);

    seq = poseGraph.create(&controller, adja, postures, offset);
  }

  RLOG(0, "Run:\n\nbin/Rcs -m 5 -f cPoseGraph.xml -algo 1 "
       "-setDefaultStateFromInit -lambda 1.0e-12\n\n");

  MatNd_destroy(postures);
  delete seq;
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  int mode = 2;
  CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is %d)",
                   RcsLogLevel);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  switch (mode)
  {
    case 0:
      printf("1\tPose Graph creation test (old)\n");
      printf("2\tPose Graph creation test (new generic solution)\n");
      break;

    case 1:
      testPoseGraphCreation();
      break;

    case 2:
      testGenericPoseGraphCreation();
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

  fprintf(stderr, "Thanks for using the TestPoseGraph app\n");

  return 0;
}
