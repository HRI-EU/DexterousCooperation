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

#ifndef DC_EXAMPLEWHEELPLANNER_H
#define DC_EXAMPLEWHEELPLANNER_H

#include <ExampleBase.h>
#include <EntityBase.h>
#include <GraphicsWindow.h>
#include <IKComponent.h>
#include <GraphComponent.h>
#include <WheelPlannerComponent.h>
#include <PhysicsComponent.h>



namespace Dc
{

class ExampleWheelPlanner : public Rcs::ExampleBase
{
public:

  ExampleWheelPlanner(int argc, char** argv);
  virtual ~ExampleWheelPlanner();
  virtual void run();
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGuis();
  virtual bool initGraphics();
  virtual void step();

protected:
  std::string cfgFile;
  std::string directory;
  std::string physicsEngine;
  std::string physicsCfg;
  unsigned int speedUp, loopCount;
  size_t queueSize;
  double dt, ttc, alpha;

  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<GraphicsWindow> viewer;
  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<WheelPlannerComponent> dmpc;
  std::unique_ptr<IKComponent> ikc;
  std::unique_ptr<PhysicsComponent> pc;
  std::vector<std::unique_ptr<ComponentBase>> hwComponents;

  Dc::EntityBase entity;
  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeTrajectory;
  ES::SubscriberCollectionDecay<const MatNd*, const MatNd*>* setTaskCommand;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand;
  ES::SubscriberCollectionDecay<>* setRenderCommand;
  ES::SubscriberCollectionDecay<std::vector<double>>* setComplianceCommand;

  bool noEventGui, noSpeedCheck, noJointCheck, noCollCheck, seqSim, seqViewer,
       zigzag, pause, sync, noLimits, simpleGraphics, startViewerWithStartEvent,
       valgrind;
};

}   // namespace Dc

#endif   // DC_EXAMPLEWHEELPLANNER_H
