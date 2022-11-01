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

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_kinematics.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_Vec3d.h>

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
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph> ExamplePoseGraph_("PoseGraph", "BoxTurning");

ExamplePoseGraph::ExamplePoseGraph(int argc, char** argv) :
  ExampleIK(argc, argv)
{
}

ExamplePoseGraph::~ExamplePoseGraph()
{
}

void ExamplePoseGraph::initParameters()
{
  xmlFileName = "cExample0.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex0";
  algo = 1;
  dt = 0.002;
  initToQ0 = true;
}

void ExamplePoseGraph::parseArgs(Rcs::CmdLineParser* argP)
{
  ExampleIK::parseArgs(argP);
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

  double offset[3];
  Vec3d_set(offset, -2.5, 0.0, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = PoseGraph::posturesFromModelState(c.getGraph(), "Step");
  PoseGraph poseGraph;
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, offset);

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

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
  return s.str();
}



/*******************************************************************************
 *
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph2> ExamplePoseGraph2_("PoseGraph", "HandRail");

ExamplePoseGraph2::ExamplePoseGraph2(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph2::~ExamplePoseGraph2()
{
}

void ExamplePoseGraph2::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample1.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex1";
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

  double offset[3];
  Vec3d_set(offset, 0.0, -1.0, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, offset);

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 *
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph3> ExamplePoseGraph3_("PoseGraph", "LightTraverse");

ExamplePoseGraph3::ExamplePoseGraph3(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph3::~ExamplePoseGraph3()
{
}

void ExamplePoseGraph3::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample2.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex2";
}

bool ExamplePoseGraph3::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  std::vector<PoseGraph::StepSpec> stepSpecs;
  std::vector<PoseGraph::Adjacency> adja;
  PoseGraph::Adjacency a;

  a.bdyName = "Traverse_1";
  a.adjacencyList = {-1, 0, -1, 2, -1};
  a.originalTasks = {"Traverse"};
  a.relaxedTasks = {""};
  a.fixLastPose = true;
  adja.push_back(a);

  a.bdyName = "PowerGrasp_R_1";
  a.adjacencyList = {-1, -1, 1, -1, 3};
  a.originalTasks = {"HandPose6d"};
  a.relaxedTasks = {"HandPose4d"};
  a.fixLastPose = false;
  adja.push_back(a);

  double offset[3];
  Vec3d_set(offset, 0.0, 1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, offset);

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Creating box pole sliding problem
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph4> ExamplePoseGraph4_("PoseGraph", "BoxOnPole_1");

ExamplePoseGraph4::ExamplePoseGraph4(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph4::~ExamplePoseGraph4()
{
}

void ExamplePoseGraph4::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample3.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex3";
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

  double offset[3];
  Vec3d_set(offset, 0.0, -1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, offset);

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Creating box pole sliding problem
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph5> ExamplePoseGraph5_("PoseGraph", "BoxOnPole_2");

ExamplePoseGraph5::ExamplePoseGraph5(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph5::~ExamplePoseGraph5()
{
}

void ExamplePoseGraph5::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample4.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex4";
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

  double offset[3];
  Vec3d_set(offset, 0.0, -1.5, 0.0);

  ControllerBase c(xmlFileName);
  MatNd* postures = NULL;
  PoseGraph poseGraph;
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, offset);

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Step sequence
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph6> ExamplePoseGraph6_("PoseGraph", "StepSequence");

ExamplePoseGraph6::ExamplePoseGraph6(int argc, char** argv) :
  ExamplePoseGraph(argc, argv), nSteps(8)
{
}

ExamplePoseGraph6::~ExamplePoseGraph6()
{
}

void ExamplePoseGraph6::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample5.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex5";
  nSteps = 8;
}

void ExamplePoseGraph6::parseArgs(Rcs::CmdLineParser* argP)
{
  ExampleIK::parseArgs(argP);
  argP->getArgument("-nSteps", &nSteps, "Number of steps (default is %d)", nSteps);
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
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, Vec3d_zeroVec());

  controller = res.controller;
  a_des = res.activation;


  // Here we set the task vector a bit different and write it out. To see
  // the model converge, start the Rcs mode 5 with the command line
  // option -xDesFile x_des.dat
#if 1
  MatNd* x = MatNd_create(res.controller->getTaskDim(), 1);
  res.controller->computeX(x);
  int idx = res.controller->getTaskArrayIndex(res.controller->getNumberOfTasks()-1);
  MatNd_set(x, idx, 0, 5.0);
  MatNd_set(x, idx+5, 0, M_PI);

  idx = res.controller->getTaskArrayIndex(1);
  MatNd_set(x, idx+1, 0, 2.0);
  MatNd_set(x, idx+5, 0, -M_PI);
  MatNd_toFile(x, "x_des.dat");

  RLOG(0, "Converging to task constaints");
  MatNd* a_des = MatNd_create(res.controller->getNumberOfTasks(), 1);
  MatNd_setElementsTo(a_des, 1.0);
  PoseGraph::convergeTaskConstraints(res.controller, a_des, x);
  res.controller->toXML("cPoseGraph.xml", a_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x);
#endif

  ExampleIK::initAlgo();

  return true;
}



/*******************************************************************************
 * Cylinder on table
 ******************************************************************************/
static Rcs::ExampleFactoryRegistrar<ExamplePoseGraph7> ExamplePoseGraph7_("PoseGraph", "CylinderOnTable");

ExamplePoseGraph7::ExamplePoseGraph7(int argc, char** argv) :
  ExamplePoseGraph(argc, argv)
{
}

ExamplePoseGraph7::~ExamplePoseGraph7()
{
}

void ExamplePoseGraph7::initParameters()
{
  ExamplePoseGraph::initParameters();
  xmlFileName = "cExample6.xml";
  directory = "config/xml/DexterousCooperation/PoseGraph/ex6";
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
  PoseGraph::Result res = poseGraph.create(&c, adja, stepSpecs, postures, Vec3d_zeroVec());

  controller = res.controller;
  a_des = res.activation;

  controller->toXML("cPoseGraph.xml", res.activation);

  ExampleIK::initAlgo();

  return true;
}






}   // namespace tropic
