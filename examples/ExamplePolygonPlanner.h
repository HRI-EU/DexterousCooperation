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

#ifndef DC_EXAMPLEPPOLYGONPLANNER_H
#define DC_EXAMPLEPPOLYGONPLANNER_H


#include <ExampleBase.h>
#include <EntityBase.h>
#include <IKComponent.h>
#include <GraphComponent.h>
#include <GraphicsWindow.h>
#include <TrajectoryComponent.h>
#include <PolygonObjectPlanner.h>
#include <RebaComponent.h>
#include <TaskGuiComponent.h>
#include <PhysicsComponent.h>


namespace Dc
{
class ExamplePolygonPlanner : public Rcs::ExampleBase
{
public:
  ExamplePolygonPlanner(int argc, char** argv);
  virtual ~ExamplePolygonPlanner();
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual bool initGuis();
  virtual void step();
  virtual void run();
  virtual std::string help();

protected:
  std::string cfgFile;
  std::string directory;
  std::string physicsEngine;
  std::string physicsCfg;
  std::string polyBaseDir;
  std::string polyDir;

  Dc::EntityBase entity;
  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeTrajectory;
  ES::SubscriberCollectionDecay<const MatNd*, const MatNd*>* setTaskCommand;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand;
  ES::SubscriberCollectionDecay<>* setRenderCommand;

  double dt, dPhi, ttc;
  unsigned int speedUp, loopCount;
  int polyDirIdx;

  std::unique_ptr<Rcs::ControllerBase> controller;
  GraphicsWindow* viewer;
  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<IKComponent> ikc;
  std::unique_ptr<TrajectoryComponent> trajC;
  std::unique_ptr<PolygonObjectPlanner> pop;
  std::unique_ptr<RebaComponent> rebaC;
  std::unique_ptr<TaskGuiComponent> taskGui;
  std::unique_ptr<PhysicsComponent> pc;

  bool withGui, noEventGui, noSpeedCheck, noJointCheck, noCollCheck, seqSim,
       seqViewer, zigzag, pause, sync, noLimits, simpleGraphics, valgrind,
       polyDebug, physics, eternalTest, skipTrajectoryCheck;


};


}   // namespace


#endif
