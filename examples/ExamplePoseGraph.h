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

#ifndef RCS_EXAMPLEPOSEGRAPH_H
#define RCS_EXAMPLEPOSEGRAPH_H

#include <PoseGraph.h>

#include <ExampleInvKin.h>


extern "C" {
  void DcExampleInfo();
}

namespace Rcs
{

class ExamplePoseGraph : public ExampleIK
{
public:

  ExamplePoseGraph(int argc, char** argv);
  virtual ~ExamplePoseGraph();
  virtual bool initParameters();
  virtual bool parseArgs(CmdLineParser* parser);
  virtual bool initAlgo();
  virtual std::string help();
  virtual void run();
  virtual void handleKeys();

  PoseGraph::SequenceAlgo seqAlgo;
  std::string sequenceAlgorithm;
  PoseGraph::Result result;
  HTr subGraphOffset;
  bool animateTrajectory;
};

class ExamplePoseGraph2 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph2(int argc, char** argv);
  virtual ~ExamplePoseGraph2();
  virtual bool initParameters();
  virtual bool initAlgo();
};

class ExamplePoseGraph3 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph3(int argc, char** argv);
  virtual ~ExamplePoseGraph3();
  virtual bool initParameters();
  virtual bool initAlgo();
};

class ExamplePoseGraph4 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph4(int argc, char** argv);
  virtual ~ExamplePoseGraph4();
  virtual bool initParameters();
  virtual bool initAlgo();
};

class ExamplePoseGraph5 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph5(int argc, char** argv);
  virtual ~ExamplePoseGraph5();
  virtual bool initParameters();
  virtual bool initAlgo();
};

class ExamplePoseGraph6 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph6(int argc, char** argv);
  virtual ~ExamplePoseGraph6();
  virtual bool initParameters();
  virtual bool initAlgo();
  virtual bool parseArgs(CmdLineParser* parser);

  int nSteps;
};

class ExamplePoseGraph7 : public ExamplePoseGraph
{
public:

  ExamplePoseGraph7(int argc, char** argv);
  virtual ~ExamplePoseGraph7();
  virtual bool initParameters();
  virtual bool initAlgo();
};

}   // namespace Rcs

#endif   // RCS_EXAMPLEPOSEGRAPH_H
