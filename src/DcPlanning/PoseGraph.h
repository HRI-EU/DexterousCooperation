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

#ifndef RCS_POSEGRAPH_H
#define RCS_POSEGRAPH_H

#include <ControllerBase.h>


namespace Rcs
{






class PoseGraph
{
public:

  struct Adjacency
  {
    std::string bdyName;   // without suffix
    std::vector<int> adjacencyList;
    std::string originalTask, relaxedTask;
    bool fixFirstPose;
    bool fixLastPose;

    Adjacency() : fixFirstPose(true), fixLastPose(true)
    {

    }
  };

  ControllerBase* create(const ControllerBase* src,
                         const std::vector<Adjacency>& adjacencies,
                         const MatNd* postures,
                         const double offset[3]);

protected:

  ControllerBase* createGraph(const ControllerBase* controller,
                              const MatNd* postures,
                              const double offset[3]);

  void eraseInactiveTasks(ControllerBase* controller, MatNd* activation);

  void linkBodyJoints(ControllerBase* controller, const Adjacency& connection,
                      MatNd* activation);
  void relax(ControllerBase* controller, const Adjacency& connection,
             MatNd* activation);

  std::vector<size_t> getTasksForEffector(const ControllerBase* controller,
                                          const std::string& effectorName,
                                          const MatNd* activation) const;
};




}   // namespace Rcs

#endif // RCS_POSEGRAPH_H
