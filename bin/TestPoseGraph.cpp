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
#include <ExampleInvKin.h>
#include <ExamplePoseGraph.h>
#include <KeyCatcher.h>

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <EulerConstraint.h>
#include <PolarConstraint.h>
#include <PoseConstraint.h>
#include <VectorConstraint.h>

#include <ConstraintFactory.h>
#include <ControllerBase.h>
#include <IkSolverRMR.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_guiFactory.h>
#include <Rcs_graphParser.h>
#include <SegFaultHandler.h>

#include <csignal>
#include <array>


RCS_INSTALL_ERRORHANDLERS

using namespace Rcs;

bool runLoop = true;




/*******************************************************************************
 *
 ******************************************************************************/
void setValueTaskXfromFile(MatNd* x,
                           const Rcs::ControllerBase* controller,
                           const char* name,
                           const char* path)
{
  int task_id, task_dim;
  task_id = controller->getTaskIndex(name);
  task_dim = controller->getTaskDim(task_id);

  double v;
  MatNd* temp_x = MatNd_create(task_dim,1);
  char filename[100];
  strcpy(filename,path);
  strcat(filename,name);
  bool suc = MatNd_fromFile(temp_x, filename);

  for (size_t i=0; i<task_dim; ++i)
  {
    v = MatNd_get(temp_x, i, 0);
    MatNd_set(x, controller->getTaskArrayIndex(name)+i, 0, v);
  }
  MatNd_destroy(temp_x);
}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<tropic::ConstraintSet>
createTrajectory(const ControllerBase* controller,
                 unsigned int nSteps,
                 const MatNd* a_des,
                 const MatNd* x_des)
{
  std::shared_ptr<tropic::ConstraintSet> tSet = std::make_shared<tropic::ConstraintSet>();

  for (unsigned int step=0; step<nSteps; ++step)
  {
    const double duration = 1.0;
    const double t_start = duration*step;
    const double hrz = 0.25*duration;

    std::shared_ptr<tropic::ActivationSet> aSet = std::make_shared<tropic::ActivationSet>();

    for (unsigned int tsk=0; tsk<controller->getNumberOfTasks(); ++tsk)
    {
      const Rcs::Task* task = controller->getTask(tsk);
      const bool isActive = MatNd_get(a_des, tsk, 0) > 0.0 ? true : false;
      aSet->addActivation(t_start, isActive, hrz, task->getName());

      const size_t xIdx = controller->getTaskArrayIndex(tsk);
      const double* x = MatNd_getElePtr(x_des, step, xIdx);

      if (task->getClassName()=="XYZ")
      {
        aSet->add(std::make_shared<tropic::PositionConstraint>(t_start+duration, x, task->getName()));
      }
      else if (task->getClassName()=="ABC")
      {
        aSet->add(std::make_shared<tropic::EulerConstraint>(t_start+duration, x, task->getName()));
      }
      else if (task->getClassName()=="POLAR")
      {
        aSet->add(std::make_shared<tropic::PolarConstraint>(t_start+duration, x[0], x[1], task->getName()));
      }
      // else if (task->getClassName()=="XYZABCxxx")
      // {
      //   aSet->add(std::make_shared<tropic::PoseConstraint>(t_start+duration, x, task->getName()));
      // }
      else
      {
        RLOG(0, "Unsupported task class: \"%s\" - using VectorConstraint",
             task->getClassName().c_str());
        std::vector<double> coords = std::vector<double>(x, x+task->getDim());
        RCHECK(coords.size()==task->getDim());
        aSet->add(std::make_shared<tropic::VectorConstraint>(t_start+duration, coords, task->getName()));
      }

    }   // for (unsigned int tsk=0; tsk<nt; ++tsk)

    tSet->add(aSet);

  }   // for (unsigned int step=0; step<nSteps; ++step)

  tSet->toXML("traj.xml");

  return tSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<tropic::ConstraintSet>
createTrajectory(ControllerBase* cSingle,
                 const ControllerBase* cSeq)
{
  // Compute number of steps
  const RcsBody* lb = RcsBody_getLastLeaf(cSeq->getGraph(),
                                          RcsBody_last(cSeq->getGraph()));
  RCHECK(lb);
  const char* suffix = strrchr(lb->name, '_');
  RCHECK(suffix);
  suffix++;
  RLOG(0, "Suffix is \"%s\"", suffix);
  int nSteps = atoi(suffix) + 1;
  RLOG(0, "nSteps = %d", nSteps);

  // Extract graph for each transition
  RcsGraph** graphs = RNALLOC(nSteps, RcsGraph*);
  std::string rootName = cSingle->getGraph()->bodies[0].name;
  MatNd* x_des_i = MatNd_create(cSingle->getTaskDim(), 1);
  MatNd* x_des = MatNd_create(cSingle->getTaskDim(), 1);
  MatNd* a_des = MatNd_create(cSingle->getNumberOfTasks(), 1);
  MatNd* q0 = MatNd_createLike(cSingle->getGraph()->q);
  RcsGraph_getDefaultState(cSingle->getGraph(), q0);
  cSingle->readActivationsFromXML(a_des);

  RcsGraph* orgControllerGraph = cSingle->getGraph();
  graphs[0] = RcsGraph_cloneSubGraph(cSeq->getGraph(), rootName.c_str());
  RcsGraph_changeDefaultState(graphs[0], q0);
  cSingle->setGraph(graphs[0], false);
  cSingle->computeX(x_des_i);
  MatNd_copy(x_des, x_des_i);

  for (int i=1; i<nSteps; ++i)
  {
    std::string rootName_i = rootName + "_" + std::to_string(i);
    graphs[i] = RcsGraph_cloneSubGraph(cSeq->getGraph(), rootName_i.c_str());
    RcsGraph_changeDefaultState(graphs[i], q0);
    RcsGraph_changeSuffix(graphs[i], std::string("_" + std::to_string(i)).c_str(), NULL);
    RcsGraph_toXML(graphs[i], std::string("g"+rootName_i+".xml").c_str());
    cSingle->setGraph(graphs[i], false);
    cSingle->computeX(x_des_i);
    MatNd_appendRows(x_des, x_des_i);
  }

  MatNd_reshape(x_des, nSteps, cSingle->getTaskDim());

  std::shared_ptr<tropic::ConstraintSet> tSet;
  tSet = createTrajectory(cSingle, nSteps, a_des, x_des);

  // Clean up
  for (int i=0; i<nSteps; ++i)
  {
    RcsGraph_destroy(graphs[i]);
  }

  RFREE(graphs);
  MatNd_destroy(x_des_i);
  MatNd_destroy(x_des);
  MatNd_destroy(a_des);
  MatNd_destroy(q0);

  return tSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
class ExampleIK_PoseGraph : public ExampleIK
{
public:

  ExampleIK_PoseGraph(int argc, char** argv) : ExampleIK(argc, argv)
  {
  }

  bool initParameters()
  {
    xmlFileName = "cPoseGraph.xml";
    directory = "";
    algo = 1;
    dt = 0.002;
    initToQ0 = true;

    return true;
  }

  // Takes ownership of c and activation.
  void setController(ControllerBase* c, MatNd* activation)
  {
    controller = c;
    a_des = activation;
  }

  bool initAlgo()
  {
    if (nomutex)
    {
      mtx = NULL;
    }

    //controller = new ControllerBase(xmlFileName.c_str());

    if (initToQ0)
    {
      MatNd* q_init = MatNd_createLike(controller->getGraph()->q);
      RcsGraph_getInitState(controller->getGraph(), q_init);
      RcsGraph_changeDefaultState(controller->getGraph(), q_init);
      MatNd_destroy(q_init);
    }

    // if (constraintIK == true)
    // {
    //   ikSolver = new Rcs::IkSolverConstraintRMR(controller);
    // }
    // else
    {
      ikSolver = new Rcs::IkSolverRMR(controller);
    }

    dq_des = MatNd_create(controller->getGraph()->dof, 1);
    q_dot_des = MatNd_create(controller->getGraph()->dof, 1);
    //a_des = MatNd_create(controller->getNumberOfTasks(), 1);
    x_curr = MatNd_create(controller->getTaskDim(), 1);
    x_physics = MatNd_create(controller->getTaskDim(), 1);
    x_des = MatNd_create(controller->getTaskDim(), 1);
    x_des_f = MatNd_create(controller->getTaskDim(), 1);
    dx_des = MatNd_create(controller->getTaskDim(), 1);
    dH = MatNd_create(1, controller->getGraph()->dof);
    MatNd_reshape(dH, 1, controller->getGraph()->nJ);

    //controller->readActivationsFromXML(a_des);
    //MatNd_setElementsTo(a_des, 1.0);
    controller->computeX(x_curr);
    MatNd_copy(x_des, x_curr);
    MatNd_copy(x_des_f, x_curr);
    MatNd_copy(x_physics, x_curr);

    // Body for static effort null space gradient
    effortBdy = RcsGraph_getBodyByName(controller->getGraph(),
                                       effortBdyName.c_str());
    F_effort = MatNd_create(4, 1);   // 4-th element is gain

    return true;
  }


};

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

    //int nErr = RcsGraph_fprintModelState(stdout, controller->getGraph(), controller->getGraph()->q, "Step", i);

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

static void testGenericPoseGraphCreation(int argc, char** argv)
{
  CmdLineParser argP;
  std::string directory = "config/xml/DexterousCooperation/PoseGraph";
  std::string example = "ex0";
  argP.getArgument("-dir", &directory, "Configuration file directory "
                   "(default is %s)", directory.c_str());
  argP.getArgument("-example", &example, "Example number (default is %s)",
                   example.c_str());
  bool noOffset = argP.hasArgument("-noOffset", "Offset between steps is zero");

  std::string seqAlgo = "Coupled";
  argP.getArgument("-seqAlgo", &seqAlgo, "Sequence connection algorithm. "
                   "Options: Coupled (default), naiiveFirst, naiiveAvg, duplicated");
  PoseGraph::SequenceAlgo salgo = PoseGraph::Coupled;
  if (seqAlgo=="naiiveFirst")
  {
    RLOG(0, "PoseGraph::SequenceAlgo is NaiiveFirstPose");
    salgo = PoseGraph::NaiiveFirstPose;
  }
  else if (seqAlgo=="naiiveAvg")
  {
    RLOG(0, "PoseGraph::SequenceAlgo is NaiiveSequence");
    salgo = PoseGraph::NaiiveSequence;
  }
  else if (seqAlgo=="duplicated")
  {
    RLOG(0, "PoseGraph::SequenceAlgo is Duplicated");
    salgo = PoseGraph::Duplicated;
  }
  else
  {
    RLOG(0, "PoseGraph::SequenceAlgo is Coupled");
  }

  directory += "/";
  directory += example;

  Rcs_addResourcePath(directory.c_str());

  if (argP.hasArgument("-h"))
  {
    RLOG(0, "Run:\n\nbin/TestPoseGraph -m 2 "
         "-example <ex0, ex1, ex2, ex3, ...>\n\n");
    return;
  }

  PoseGraph poseGraph;
  std::vector<PoseGraph::Adjacency> adja;
  MatNd* postures = NULL;
  ControllerBase* controller = NULL;
  PoseGraph::Result res;
  PoseGraph::Adjacency a;
  std::vector<PoseGraph::StepSpec> stepSpec;
  double offset[3];
  Vec3d_setZero(offset);

  if (example=="ex0")
  {
    controller = new ControllerBase("cExample0.xml");

    PoseGraph::TaskSpec tskSpec;
    tskSpec.taskName = "Phi_Box";
    tskSpec.x_des = {0.0, 0.0, 0.0};
    tskSpec.active = true;

    PoseGraph::StepSpec stepInfo;
    stepInfo.taskSpec.push_back(tskSpec);

    stepSpec.push_back(stepInfo);   // Step 1
    stepSpec.push_back(stepInfo);   // Step 2

    tskSpec.x_des = {0.0, 0.0, M_PI_2};

    stepSpec.push_back(stepInfo);   // Step 3
    stepSpec.push_back(stepInfo);   // Step 4
    stepSpec.push_back(stepInfo);   // Step 5

    tskSpec.x_des = {0.0, 0.0, M_PI_2};

    stepSpec.push_back(stepInfo);   // Step 6
    stepSpec.push_back(stepInfo);   // Step 7



    a.bdyName = "Box_v";
    a.adjacencyList = {-1, 0, -1, 2, 3, -1, 5};
    a.originalTasks = {"Phi_Box"};
    a.relaxedTasks = {"Polar_Box"};
    //adja.push_back(a);

    a.bdyName = "PowerGrasp_R";
    a.adjacencyList = {-1, 0, 1, -1, 3, 4, -1};
    a.originalTasks = {"ABC_R"};
    a.relaxedTasks = {"Polar_R"};
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "PowerGrasp_L";
    a.adjacencyList = {-1, -1, 1, 2, -1, 4, 5};
    a.originalTasks = {"ABC_L"};
    a.relaxedTasks = {"Polar_L"};
    a.fixLastPose = false;
    adja.push_back(a);

    if (!noOffset)
    {
      Vec3d_set(offset, -2.5, 0.0, 0.0);
    }

    // std::vector<double> times{5, 11, 23, 29, 35, 53, 59};
    // auto ts = tropic::ConstraintFactory::create(directory + "/trajectory.xml");
    // RLOG(1, "DynamicPoseGraph end time: %f", ts->getEndTime());
    //postures = createPoseSequence(&controller, times, ts);
    postures = PoseGraph::posturesFromModelState(controller->getGraph(), "Step");

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);
  }
  else if (example=="ex1")
  {
    RLOG(0, "Creating box surface sliding problem");
    controller = new ControllerBase("cExample1.xml");

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1, 4, -1, 6, -1};
    a.originalTasks = {"Object"};
    a.relaxedTasks = {"ObjectYZ"};
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3, -1, 5, -1, 7};
    a.originalTasks = {"Hand Pose"};
    a.relaxedTasks = {"Hand Dist"};
    a.fixLastPose = false;
    adja.push_back(a);

    if (!noOffset)
    {
      Vec3d_set(offset, 0.0, -1.0, 0.0);
    }

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);
  }
  else if (example=="ex2")
  {
    RLOG(0, "Creating light traverse sliding problem");
    controller = new ControllerBase("cExample2.xml");

    a.bdyName = "Traverse_1";
    a.adjacencyList = {-1, 0, -1, 2, -1, 4, -1};
    a.originalTasks = {"Traverse"};
    a.relaxedTasks = {""};
    a.fixLastPose = true;
    adja.push_back(a);

    a.bdyName = "PowerGrasp_R_1";
    a.adjacencyList = {-1, -1, 1, -1, 3, -1, 5};
    a.originalTasks = {"HandPos", "HandOri"};
    a.relaxedTasks = {"HandPosYZ", "HandOriPolar"};
    a.fixLastPose = false;
    adja.push_back(a);

    if (!noOffset)
    {
      Vec3d_set(offset, 0.0, 1.5, 0.0);
    }

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);

#if 1
    int tidx = res.controller->getTaskArrayIndex("Traverse_6");
    RCHECK(tidx!=-1);
    MatNd* x_curr = MatNd_create(res.controller->getTaskDim(), 1);
    res.controller->computeX(x_curr);
    MatNd_set(x_curr, tidx, 0, -3.5);
    Rcs::PoseGraph::convergeTaskConstraints(res.controller, res.activation, x_curr, 200);
    MatNd_destroy(x_curr);
#endif
  }
  else if (example=="ex3")
  {
    RLOG(0, "Creating box pole sliding problem");
    controller = new ControllerBase("cExample3.xml");

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1};
    a.originalTasks = {"Box"};
    a.relaxedTasks = {""};
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3};
    a.originalTasks = {"Hand-Box pose"};
    a.relaxedTasks = {"Hand-Box distance"};
    a.fixLastPose = false;
    adja.push_back(a);

    if (!noOffset)
    {
      Vec3d_set(offset, 0.0, -1.5, 0.0);
    }

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);
  }
  else if (example=="ex4")
  {
    RLOG(0, "Creating box pole sliding problem");
    controller = new ControllerBase("cExample4.xml");

    a.bdyName = "Box";
    a.adjacencyList = {-1, 0, -1, 2, -1, 4, -1};
    // a.adjacencyList = {-1, 0, -1};
    a.originalTasks = {"Box"};
    a.relaxedTasks = {""};
    a.fixFirstPose = false;
    a.fixLastPose = false;
    adja.push_back(a);

    a.bdyName = "LeftHand";
    a.adjacencyList = {-1, -1, 1, -1, 3, -1, 5};
    // a.adjacencyList = {-1, -1, 1};
    a.originalTasks = {"Hand-Box pose"};
    a.relaxedTasks = {"Hand-Box distance"};
    a.fixFirstPose = true;
    a.fixLastPose = false;
    adja.push_back(a);

    if (!noOffset)
    {
      Vec3d_set(offset, 0.0, -1.5, 0.0);
    }

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);
  }
  else if (example=="ex5")
  {
    int nSteps = 8;
    argP.getArgument("-nSteps", &nSteps, "Number of steps (default is %d)", nSteps);
    RLOG(0, "Creating step sequence example with %d steps", nSteps);
    controller = new ControllerBase("cExample5.xml");
    const bool evenNoSteps = (nSteps%2==0) ? true : false;

    // For 8 steps, it must be: a.adjacencyList = {-1, 0, -1, 2, -1, 4, -1, 6};
    a.taskName = "Right Foot 6D";
    for (int i=0; i<nSteps; ++i)
    {
      int prevStep = (i%2==0) ? -1 : i-1;
      RLOG(1, "Step %d: connection is %d", i, prevStep);
      a.adjacencyList.push_back(prevStep);
    }
    a.originalTasks = {"Right Foot 6D"};
    a.relaxedTasks = {"Right Foot Slide"};
    a.fixFirstPose = false;   // We start with a step of the right foot
    a.fixLastPose = evenNoSteps ? false : true;
    adja.push_back(a);


    //For 8 steps, it must be: a.adjacencyList = {-1, -1, 1, -1, 3, -1, 5, -1};
    a.adjacencyList.clear();
    a.taskName = "Left Foot 6D";
    for (int i=0; i<nSteps; ++i)
    {
      int prevStep = (i%2==0) ? i-1 : -1;
      a.adjacencyList.push_back(prevStep);
      RLOG(1, "Step %d: connection is %d", i, prevStep);
    }
    a.originalTasks = {"Left Foot 6D"};
    a.relaxedTasks = {"Left Foot Slide"};
    a.fixFirstPose = true;
    a.fixLastPose = evenNoSteps ? true : false;
    adja.push_back(a);

    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);

    // Here we set the task vector a bit different and write it out.
    MatNd* x = MatNd_create(res.controller->getTaskDim(), 1);
    res.controller->computeX(x);
    int idx = res.controller->getTaskArrayIndex("Left Foot 6D_7");
    if (idx!=-1)
    {
      MatNd_set(x, idx, 0, 5.0);
      MatNd_set(x, idx+5, 0, M_PI);
    }
    else
    {
      RLOG(0, "Couldn't set goal for task Left Foot 6D_7");
    }


    idx = res.controller->getTaskArrayIndex("Left Foot 6D");
    if (idx!=-1)
    {
      MatNd_set(x, idx+1, 0, 2.0);
      MatNd_set(x, idx+5, 0, -M_PI);
    }
    else
    {
      RLOG(0, "Couldn't set goal for task Left Foot 6D");
    }


    Rcs::PoseGraph::convergeTaskConstraints(res.controller, res.activation, x, 200);
    MatNd_destroy(x);
  }
  else if (example=="ex6")
  {
    RLOG(0, "Creating cylinder on table");
    controller = new ControllerBase("cExample6.xml");

    a.taskName = "Hand Pose";
    a.adjacencyList = {-1, 0 };
    a.originalTasks = {"Hand Pose"};
    a.relaxedTasks = {"Hand Pose5"};
    a.fixFirstPose = false;
    a.fixLastPose = false;
    adja.push_back(a);

    postures = PoseGraph::posturesFromModelState(controller->getGraph(), "Step");
    res = poseGraph.create(controller, adja, stepSpec, postures, offset, salgo);
  }
  else if (example=="ex8ciii")
  {
    // where to place milk to pick, for human to pur and
    // where will the human place the milk after
    // and how to rotate mug to alow human to pick it up
    RLOG(0, "Creating table top scenario with milk");
    ControllerBase controller("cExample8ciii.xml");
    PoseGraph::Adjacency a;

    a.bdyName = "Mug";
    a.adjacencyList = {-1, 0, 1, 2, 3, -1};
    a.originalTasks = {"Mug_poseZAB"};
    a.relaxedTasks = {" "};
    adja.push_back(a);


    a.bdyName = "Milk";
    a.adjacencyList.clear();
    a.adjacencyList = {-1, 0, -1, -1, -1, 4};
    a.originalTasks = {"Milk_pose_XYZABC"};
    a.relaxedTasks = {"Milk_pose_ZAB"};
    a.fixFirstPose = false;
    a.fixLastPose = false;
    adja.push_back(a);


    // a.bdyName = "Mug2";
    // a.adjacencyList = {-1, 0, 1, -1, 3, 4};
    // a.originalTask = "Mug_poseZAB2";
    // a.relaxedTask = " ";
    // adja.push_back(a);


    a.bdyName = "RightHandMilk_i";
    a.adjacencyList = {-1, -1, 1, 2, 3, -1};
    a.originalTasks = {"G_Milk_XYZC"};
    a.relaxedTasks = {" "};
    a.fixFirstPose = true;
    adja.push_back(a);


    a.bdyName = "Mug3";
    a.adjacencyList = {-1, 0, 1, 2, 3, -1};
    a.originalTasks = {"Mug_poseZAB3"};
    a.relaxedTasks = {" "};
    adja.push_back(a);

    std::vector<std::array<double, 3>> offset_list;
    std::array<double, 3> offset;
    offset = {0.0, 0.0, 0.0 };
    offset_list.push_back(offset);

    offset = {0.0, 1.5, 0.0 };
    offset_list.push_back(offset);

    offset = {0.0, 3.0, 0.0 };
    offset_list.push_back(offset);

    offset = {0.0, 4.5, 0.0 };
    offset_list.push_back(offset);

    offset = {0.0, 6.0, 0.0 };
    offset_list.push_back(offset);

    offset = {0.0, 7.5, 0.0 };
    offset_list.push_back(offset);

    double offs[3];
    Vec3d_set(offs, 0.0, 1.5, 0.0);

    res = poseGraph.create(&controller, adja, stepSpec, postures, offs, salgo);

    // Create controller
    static Rcs::ControllerBase cntr("cPoseGraph.xml");

#if 1
    // modify x vector
    MatNd* x = MatNd_create(cntr.getTaskDim(), 1);
    cntr.computeX(x);

    // update path to files
    std::string path2taskvalfiles = directory + "/taskValues/";

    // initial
    setValueTaskXfromFile(x, &cntr, "RH_G_XYZ_1", path2taskvalfiles.c_str());
    setValueTaskXfromFile(x, &cntr, "RH_G_XYZ_2", path2taskvalfiles.c_str());
    setValueTaskXfromFile(x, &cntr, "RH_G_XYZ_3", path2taskvalfiles.c_str());
    setValueTaskXfromFile(x, &cntr, "RH_G_XYZ_4", path2taskvalfiles.c_str());

    // // ...
    setValueTaskXfromFile(x, &cntr, "G_Milk_XYZC_1", path2taskvalfiles.c_str());
    setValueTaskXfromFile(x, &cntr, "Pur_XYZABC_2", path2taskvalfiles.c_str());
    setValueTaskXfromFile(x, &cntr, "Pur_XYZAC3_3", path2taskvalfiles.c_str());

    setValueTaskXfromFile(x, &cntr, "Milk_pose_ZAB_4", path2taskvalfiles.c_str());



    // modify a vector
    MatNd* a_des = MatNd_create(cntr.getNumberOfTasks(), 1);
    cntr.readActivationsFromXML(a_des);

    // activate pick milk
    int task_id;
    task_id = cntr.getTaskIndex("G_Milk_XYZC_1");
    MatNd_set(a_des, task_id, 0.0, 1.0);
    task_id = cntr.getTaskIndex("RH_G_XYZ_1");
    MatNd_set(a_des, task_id, 0.0, 1.0);

    // // // activate purring
    task_id = cntr.getTaskIndex("Pur_XYZABC_2");
    MatNd_set(a_des, task_id, 0.0, 1.0);
    task_id = cntr.getTaskIndex("RH_G_XYZ_2");
    MatNd_set(a_des, task_id, 0.0, 1.0);
    // // deactivate relaxed milk pose
    task_id = cntr.getTaskIndex("Milk_pose_ZAB_2");
    MatNd_set(a_des, task_id, 0.0, 0.0);

    // // // activate purring 3
    task_id = cntr.getTaskIndex("Pur_XYZAC3_3");
    MatNd_set(a_des, task_id, 0.0, 1.0);
    task_id = cntr.getTaskIndex("RH_G_XYZ_3");
    MatNd_set(a_des, task_id, 0.0, 1.0);
    // // deactivate relaxed milk pose
    task_id = cntr.getTaskIndex("Milk_pose_ZAB_3");
    MatNd_set(a_des, task_id, 0.0, 0.0);

    // drop milk
    task_id = cntr.getTaskIndex("RH_G_XYZ_4");
    MatNd_set(a_des, task_id, 0.0, 1.0);


    RPAUSE_MSG("Hit enter to converge task constaints");
    Rcs::PoseGraph::convergeTaskConstraints(&cntr, a_des, x);
    cntr.toXML("ex8ciii/cPoseGraph.xml", a_des);
    MatNd_copy(res.activation, a_des);
    MatNd_destroy(a_des);
    MatNd_destroy(x);
    res.controller = &cntr;

#endif
  }


  else
  {
    RFATAL("Unknown example: \"%s\"", example.c_str());
  }





  res.controller->toXML("cPoseGraph.xml", res.activation);

  MatNd_destroy(postures);



  // Visualize and iterate the coupled system
  ExampleIK_PoseGraph* exampleIK = new ExampleIK_PoseGraph(argc, argv);
  exampleIK->setController(res.controller, res.activation);
  exampleIK->init(argc, argv);
  exampleIK->start();   // Stops when q is pressed in viewer window
  exampleIK->stop();

  // We create the trajectory after some convergence.
  createTrajectory(controller, res.controller);

  delete exampleIK;

#if defined (_MSC_VER)
  RLOG(0, "Run:\n\nbin\\Release\\Rcs -m 5 -f cPoseGraph.xml -algo 1 "
       "-setDefaultStateFromInit -lambda 1.0e-12\n\n");
#else
  RLOG(0, "Run:\n\nbin/Rcs -m 5 -f cPoseGraph.xml -algo 1 "
       "-setDefaultStateFromInit -lambda 1.0e-12\n\n");


  RLOG(0, "Run:\n\nbin/ExampleRunnerAll -m 2 -c Trajectory -e InverseKinematics -dir config/xml/DexterousCooperation/PoseGraph/ex2/ -f cExample2.xml -lambda .001");
#endif
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
      testGenericPoseGraphCreation(argc, argv);
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
