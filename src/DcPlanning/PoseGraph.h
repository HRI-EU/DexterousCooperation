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

#ifndef DC_POSEGRAPH_H
#define DC_POSEGRAPH_H

#include <ControllerBase.h>


namespace Rcs
{






class PoseGraph
{
public:

  struct TaskSpec
  {
    std::string taskName;
    std::vector<double> x_des;
    bool active;
    bool beforeRelaxation;
    bool afterRelaxation;

    TaskSpec() : active(false), beforeRelaxation(false), afterRelaxation(false)
    {
    }
  };



  struct StepSpec
  {
    std::vector<TaskSpec> taskSpec;
    std::vector<double> posture;
  };



  struct Adjacency
  {
    std::string bdyName;   // without suffix
    std::string taskName;  // without suffix
    std::vector<int> adjacencyList;
    std::vector<std::string> originalTasks;
    std::vector<std::string> relaxedTasks;
    bool fixFirstPose;
    bool fixLastPose;

    Adjacency() : fixFirstPose(true), fixLastPose(true)
    {
    }
  };

  struct Result
  {
    ControllerBase* cSingle;
    ControllerBase* controller;
    MatNd* activation;
    Result() : cSingle(NULL), controller(NULL), activation(NULL)
    { }
  };

  typedef enum
  {
    Coupled,
    NaiiveFirstPose,
    NaiiveSequence,
    Duplicated

  } SequenceAlgo;

  Result create(const ControllerBase* src,
                const std::vector<Adjacency>& adjacencies,
                const std::vector<StepSpec>& stepSpecs,
                const MatNd* postures,
                const double offset[3],
                SequenceAlgo algo=Coupled);

  static MatNd* posturesFromModelState(const RcsGraph* graph,
                                       const std::string& mdlStateName);

  static bool convergeTaskConstraints(ControllerBase* controller,
                                      const MatNd* activation,
                                      const MatNd* x_des,
                                      int maxIter=200,
                                      double clipLimit=0.01);

protected:

  Result createNaiive(const ControllerBase* src,
                      const std::vector<Adjacency>& adjacencies,
                      const MatNd* postures,
                      const double offset[3],
                      bool avgSeq);

  Result createNaiive_1(const ControllerBase* cSingle,
                        const std::vector<Adjacency>& adjacencies,
                        const MatNd* postures,
                        const double offset[3],
                        bool avgSeq,
                        MatNd* x_relaxed);

  void createNaiive_2(ControllerBase* cNaiive,
                      const MatNd* aNaiive,
                      const MatNd* x_des);

  Result createCoupled(const ControllerBase* src,
                       const std::vector<Adjacency>& adjacencies,
                       const MatNd* postures,
                       const double offset[3],
                       bool duplicatedOnly=false);

  ControllerBase* createGraph(const ControllerBase* controller,
                              const MatNd* postures,
                              const double offset[3],
                              const MatNd* optionalInitState=NULL);

  void eraseInactiveTasks(ControllerBase* controller, MatNd* activation);

  void linkBodyJoints(ControllerBase* controller, const Adjacency& connection,
                      MatNd* activation);
  void linkTasks(ControllerBase* controller, const Adjacency& connection,
                 MatNd* activation);
  void relax(ControllerBase* controller, const Adjacency& connection,
             MatNd* activation);
  void relaxAll(ControllerBase* controller, const Adjacency& connection,
                MatNd* activation);
  void computeAverageXdes(ControllerBase* controller,
                          const Adjacency& connection,
                          MatNd* activation,
                          MatNd* x_des,
                          bool avgSeq);

  std::vector<size_t> getTasksForEffector(const ControllerBase* controller,
                                          const std::string& effectorName,
                                          const MatNd* activation) const;
};




}   // namespace Rcs

#endif // DC_POSEGRAPH_H
