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

#include "ExamplePoseGraph.h"

#include <ExampleTrajectoryIK.h>
#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_kinematics.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_Vec3d.h>
#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <EulerConstraint.h>
#include <PolarConstraint.h>
#include <VectorConstraint.h>
#include <ConstraintFactory.h>

#include <KeyCatcher.h>


void DcExampleInfo()
{
  Rcs::ExampleFactory::print();
}

namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
static std::shared_ptr<tropic::ConstraintSet>
createTrajectory(const ControllerBase* controller,
                 unsigned int nSteps,
                 const MatNd* a_des,
                 const MatNd* x_des)
{
  std::shared_ptr<tropic::ConstraintSet> tSet = std::make_shared<tropic::ConstraintSet>();

  for (unsigned int step = 0; step < nSteps; ++step)
  {
    const double duration = 1.0;
    const double t_start = duration * step;
    const double hrz = 0.25 * duration;

    std::shared_ptr<tropic::ActivationSet> aSet = std::make_shared<tropic::ActivationSet>();

    for (unsigned int tsk = 0; tsk < controller->getNumberOfTasks(); ++tsk)
    {
      const Rcs::Task* task = controller->getTask(tsk);
      const bool isActive = MatNd_get(a_des, tsk, 0) > 0.0 ? true : false;
      aSet->addActivation(t_start, isActive, hrz, task->getName());

      const size_t xIdx = controller->getTaskArrayIndex(tsk);
      const double* x = MatNd_getElePtr(x_des, step, xIdx);

      if (task->getClassName() == "XYZ")
      {
        aSet->add(std::make_shared<tropic::PositionConstraint>(t_start + duration, x, task->getName()));
      }
      else if (task->getClassName() == "ABC")
      {
        aSet->add(std::make_shared<tropic::EulerConstraint>(t_start + duration, x, task->getName()));
      }
      else if (task->getClassName() == "POLAR")
      {
        aSet->add(std::make_shared<tropic::PolarConstraint>(t_start + duration, x[0], x[1], task->getName()));
      }
      // else if (task->getClassName()=="XYZABCxxx")
      // {
      //   aSet->add(std::make_shared<tropic::PoseConstraint>(t_start+duration, x, task->getName()));
      // }
      else
      {
        RLOG(0, "Unsupported task class: \"%s\" - using VectorConstraint",
             task->getClassName().c_str());
        std::vector<double> coords = std::vector<double>(x, x + task->getDim());
        RCHECK(coords.size() == task->getDim());
        aSet->add(std::make_shared<tropic::VectorConstraint>(t_start + duration, coords, task->getName()));
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
static std::shared_ptr<tropic::ConstraintSet>
createTrajectory(ControllerBase* cSingle,
                 const ControllerBase* cSeq,
                 const HTr* offset)
{
  RLOG(0, "Creating trajectory");
  RCHECK(cSingle);
  RCHECK(cSeq);

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

  for (int i = 1; i < nSteps; ++i)
  {
    std::string rootName_i = rootName + "_" + std::to_string(i);
    graphs[i] = RcsGraph_cloneSubGraph(cSeq->getGraph(), rootName_i.c_str());





    // Invert the visual offset for root-level bodies. This needs improvement.
    for (unsigned int j = 0; j < graphs[i]->nBodies; ++j)
    {
      RcsBody* bdy = &graphs[i]->bodies[j];

      // These are root-level bodies
      if (bdy->parentId == -1)
      {
        double offs[3];
        Vec3d_constMul(offs, offset->org, i);
        Vec3d_subSelf(bdy->A_BP.org, offs);
        //HTr A_BP;
        //HTr_setIdentity(&A_BP);
        //Vec3d_constMul(A_BP.org, offset->org, -1.0);
        //HTr_transformSelf(&bdy->A_BP, &A_BP);

      }   // End root-level bodies

    }

    RcsGraph_setState(graphs[i], NULL, NULL);












    RcsGraph_changeDefaultState(graphs[i], q0);
    RcsGraph_changeSuffix(graphs[i], std::string("_" + std::to_string(i)).c_str(), NULL);
    RcsGraph_toXML(graphs[i], std::string("g" + rootName_i + ".xml").c_str());
    cSingle->setGraph(graphs[i], false);
    cSingle->computeX(x_des_i);
    MatNd_appendRows(x_des, x_des_i);
  }

  MatNd_reshape(x_des, nSteps, cSingle->getTaskDim());

  std::shared_ptr<tropic::ConstraintSet> tSet;
  tSet = createTrajectory(cSingle, nSteps, a_des, x_des);

  // Reset original graph, otherwise its destructor will double delete the
  // graph we have been setting to.
  cSingle->setGraph(orgControllerGraph, false);

  // Clean up
  for (int i = 0; i < nSteps; ++i)
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
RCS_REGISTER_EXAMPLE(ExamplePoseGraph, "PoseGraph", "BoxTurning");

ExamplePoseGraph::ExamplePoseGraph(int argc, char** argv) :
  ExampleIK(argc, argv), seqAlgo(PoseGraph::SequenceAlgo::Coupled),
  sequenceAlgorithm("Coupled"), animateTrajectory(false)
{
  Rcs::KeyCatcherBase::registerKey("A", "Animate trajectory");
  HTr_setIdentity(&subGraphOffset);
}

ExamplePoseGraph::~ExamplePoseGraph()
{
}

bool ExamplePoseGraph::initParameters()
{
  ExampleIK::initParameters();
  xmlFileName = "cExample0.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex0";
  algo = 1;
  dt = 0.002;
  initToQ0 = true;

  return true;
}

bool ExamplePoseGraph::parseArgs(Rcs::CmdLineParser* argP)
{
  ExampleIK::parseArgs(argP);

  argP->getArgument("-seqAlgo", &sequenceAlgorithm, "PoseGraph algorithm: "
                    "Coupled, NaiiveFirstPose, NaiiveSequence, Duplicated"
                    "(default is Coupled)");

  if (sequenceAlgorithm=="Coupled")
  {
    seqAlgo = PoseGraph::SequenceAlgo::Coupled;
  }
  else if (sequenceAlgorithm=="NaiiveFirstPose")
  {
    seqAlgo = PoseGraph::SequenceAlgo::NaiiveFirstPose;
  }
  else if (sequenceAlgorithm=="NaiiveSequence")
  {
    seqAlgo = PoseGraph::SequenceAlgo::NaiiveSequence;
  }
  else if (sequenceAlgorithm=="Duplicated")
  {
    seqAlgo = PoseGraph::SequenceAlgo::Duplicated;
  }
  else
  {
    RFATAL("Unknown sequence algorithm: %s", sequenceAlgorithm.c_str());
  }

  return true;
}

bool ExamplePoseGraph::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());


  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.bdyName = "Box_v";
  a.adjacencyList = {-1, 0, -1, 2, 3, -1, 5};
  a.originalTasks = {"Phi_Box"};
  a.relaxedTasks = {"Polar_Box"};
  adja.push_back(a);

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

  Vec3d_set(subGraphOffset.org, -2.5, 0.0, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = PoseGraph::posturesFromModelState(c.getGraph(), "Step");
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);
  MatNd_destroy(postures);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}

std::string ExamplePoseGraph::help()
{
  std::stringstream s;
  s << "  PoseGraph test\n\n";
  s << Rcs::ControllerBase::printUsageToString(xmlFileName);
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  s << "\n\tWhen pressing the A-key, the example creates a trajectory and";
  s << "\n\tanimates it in a graphics window.\n";
  return s.str();
}

void ExamplePoseGraph::handleKeys()
{
  ExampleIK::handleKeys();

  if (!kc.valid())
  {
    return;
  }

  if (kc->getAndResetKey('A'))
  {
    runLoop = false;
    animateTrajectory = true;
  }

}

void ExamplePoseGraph::run()
{
  ExampleIK::run();

  if (!animateTrajectory)
  {
    return;
  }

  // We create the trajectory after some convergence.
  auto ts = createTrajectory(result.cSingle, result.controller, &subGraphOffset);

  Rcs::CmdLineParser argP;
  char** argv;
  int argc = argP.getArgs(&argv);
  tropic::ExampleTrajectoryIK exTrj(argc, argv);
  exTrj.initParameters();
  exTrj.lambda = 0.001;
  exTrj.xmlFileName = xmlFileName;
  exTrj.directory = directory;
  RLOG(0, "Starting traj example");
  RLOG(0, "initAlgo");
  exTrj.initAlgo();
  RLOG(0, "initGraphics");
  exTrj.initGraphics();
  exTrj.initGuis();


  //auto ts = tropic::ConstraintFactory::create("traj.xml");
  exTrj.tc->addAndApply(ts, false);

  RLOG(0, "Starting traj example");
  exTrj.start();
  RLOG(0, "Done traj example");
}



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph2, "PoseGraph", "HandRail");

ExamplePoseGraph2::ExamplePoseGraph2(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph2::~ExamplePoseGraph2()
{
}

bool ExamplePoseGraph2::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample1.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex1";

  return true;
}

bool ExamplePoseGraph2::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.bdyName = "Box";
  a.adjacencyList = {-1, 0, -1, 2, -1};
  a.originalTasks = {"Object"};
  a.relaxedTasks = {"ObjectYZ"};
  adja.push_back(a);

  a.bdyName = "LeftHand";
  a.adjacencyList = {-1, -1, 1, -1, 3};
  a.originalTasks = {"Hand Pose"};
  a.relaxedTasks = {"Hand Dist"};
  a.fixLastPose = false;
  adja.push_back(a);

  Vec3d_set(subGraphOffset.org, 0.0, -1.0, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph3, "PoseGraph", "LightTraverse");

ExamplePoseGraph3::ExamplePoseGraph3(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph3::~ExamplePoseGraph3()
{
}

bool ExamplePoseGraph3::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample2.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex2";

  return true;
}

bool ExamplePoseGraph3::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.bdyName = "Traverse_1";
  a.adjacencyList = { -1, 0, -1, 2, -1, 4, -1 };
  a.originalTasks = {"Traverse"};
  a.relaxedTasks = {""};
  a.fixLastPose = true;
  adja.push_back(a);

  a.bdyName = "PowerGrasp_R_1";
  a.adjacencyList = { -1, -1, 1, -1, 3, -1, 5 };
  a.originalTasks = { "HandPos", "HandOri" }; //{"HandPose6d"};
  a.relaxedTasks = { "HandPosYZ", "HandOriPolar" }; //{"HandPose4d"};
  a.fixLastPose = false;
  adja.push_back(a);
  PoseGraph::TaskSpec tskSpec;
  tskSpec.taskName = "Traverse";
  PoseGraph::StepSpec stepInfo;
  stepInfo.taskSpec.push_back(tskSpec);

  stepSpecs.push_back(stepInfo);   // Step 2
  stepSpecs.push_back(stepInfo);   // Step 2
  stepSpecs.push_back(stepInfo);   // Step 2
  tskSpec.x_des = { -2.0, 0.0, 0.0 };
  stepSpecs.push_back(stepInfo);   // Step 2

  Vec3d_set(subGraphOffset.org, 0.0, 1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);
#if 1
  // Manually converge something
  int tidx = result.controller->getTaskArrayIndex("Traverse_6");
  RCHECK(tidx != -1);
  MatNd* x_curr = MatNd_create(result.controller->getTaskDim(), 1);
  result.controller->computeX(x_curr);
  MatNd_set(x_curr, tidx, 0, -3.5);
  Rcs::PoseGraph::convergeTaskConstraints(result.controller, result.activation, x_curr, 200);
  MatNd_destroy(x_curr);
#endif

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Creating box pole sliding problem
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph4, "PoseGraph", "BoxOnPole_1");

ExamplePoseGraph4::ExamplePoseGraph4(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph4::~ExamplePoseGraph4()
{
}

bool ExamplePoseGraph4::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample3.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex3";

  return true;
}

bool ExamplePoseGraph4::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

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

  Vec3d_set(subGraphOffset.org, 0.0, -1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Creating box pole sliding problem
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph5, "PoseGraph", "BoxOnPole_2");

ExamplePoseGraph5::ExamplePoseGraph5(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph5::~ExamplePoseGraph5()
{
}

bool ExamplePoseGraph5::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample4.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex4";

  return true;
}

bool ExamplePoseGraph5::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.bdyName = "Box";
  a.adjacencyList = {-1, 0, -1, 2, -1};
  // a.adjacencyList = {-1, 0, -1};
  a.originalTasks = {"Box"};
  a.relaxedTasks = {""};
  a.fixFirstPose = false;
  a.fixLastPose = false;
  adja.push_back(a);

  a.bdyName = "LeftHand";
  a.adjacencyList = {-1, -1, 1, -1, 3};
  // a.adjacencyList = {-1, -1, 1};
  a.originalTasks = {"Hand-Box pose"};
  a.relaxedTasks = {"Hand-Box distance"};
  a.fixLastPose = false;
  adja.push_back(a);

  Vec3d_set(subGraphOffset.org, 0.0, -1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Step sequence
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph6, "PoseGraph", "StepSequence");

ExamplePoseGraph6::ExamplePoseGraph6(int argc, char** argv) :
  ExamplePoseGraph(argc, argv), nSteps(8)
{
}

ExamplePoseGraph6::~ExamplePoseGraph6()
{
}

bool ExamplePoseGraph6::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample5.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex5";
  nSteps = 8;

  return true;
}

bool ExamplePoseGraph6::parseArgs(Rcs::CmdLineParser* argP)
{
  ExampleIK::parseArgs(argP);
  argP->getArgument("-nSteps", &nSteps, "Number of steps (default is %d)", nSteps);

  return true;
}

bool ExamplePoseGraph6::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;
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

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);


  // Here we set the task vector a bit different and write it out. To see
  // the model converge, start the Rcs mode 5 with the command line
  // option -xDesFile x_des.dat
#if 1
  MatNd* x = MatNd_create(result.controller->getTaskDim(), 1);
  result.controller->computeX(x);
  int idx = result.controller->getTaskArrayIndex(result.controller->getNumberOfTasks() - 1);
  RLOG(0, "idx 1: %d   x->m: %d", idx, x->m);
  MatNd_set(x, idx, 0, 5.0);
  MatNd_set(x, idx+5, 0, M_PI);

  idx = result.controller->getTaskArrayIndex(1);
  RLOG(0, "idx 2: %d   x->m: %d", idx, x->m);
  MatNd_set(x, idx+1, 0, 2.0);
  MatNd_set(x, idx+5, 0, -M_PI);
  MatNd_toFile(x, "x_des.dat");

  RLOG(0, "Converging to task constaints");
  MatNd* a_des = MatNd_create(result.controller->getNumberOfTasks(), 1);
  MatNd_setElementsTo(a_des, 1.0);
  PoseGraph::convergeTaskConstraints(result.controller, a_des, x);
  result.controller->toXML("cPoseGraph.xml", a_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x);
#endif

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Cylinder on table
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExamplePoseGraph7, "PoseGraph", "CylinderOnTable");

ExamplePoseGraph7::ExamplePoseGraph7(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph7::~ExamplePoseGraph7()
{
}

bool ExamplePoseGraph7::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample6.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex6";

  return true;
}

bool ExamplePoseGraph7::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.taskName = "Hand Pose";
  a.adjacencyList = {-1, 0 };
  a.originalTasks = {"Hand Pose"};
  a.relaxedTasks = {"Hand Pose5"};
  a.fixFirstPose = false;
  a.fixLastPose = false;
  adja.push_back(a);

  ControllerBase c(xmlFileName);
  MatNd* postures = PoseGraph::posturesFromModelState(c.getGraph(), "Step");
  PoseGraph poseGraph;
  result = poseGraph.create(&c, adja, stepSpecs, postures, subGraphOffset.org, seqAlgo);

  controller = result.controller;
  a_des = result.activation;

  controller->toXML("cPoseGraph.xml", result.activation);

  ExampleIK::initAlgo();

  return true;
}






}   // namespace
