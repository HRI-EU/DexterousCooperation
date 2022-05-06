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

#include "ROSSpinnerComponent.h"

#include <EntityBase.h>
#include <Rcs_macros.h>



namespace Rcs
{

ROSSpinnerComponent::ROSSpinnerComponent(EntityBase* parent, int threadCount) :
  ComponentBase(parent),
  spinner(threadCount),
  callbackUpdatePeriod(0.0)
{
  RLOG(0, "subscribe Start");
  subscribe("Start", &ROSSpinnerComponent::onStart);
  RLOG(0, "subscribe Stop");
  subscribe("Stop", &ROSSpinnerComponent::onStop);
  RLOG(0, "subscribe done");
}

ROSSpinnerComponent::~ROSSpinnerComponent()
{
}

double ROSSpinnerComponent::getCallbackUpdatePeriod() const
{
  return callbackUpdatePeriod;
}

void ROSSpinnerComponent::setCallbackUpdatePeriod(double period)
{
  callbackUpdatePeriod = period;
}

void ROSSpinnerComponent::onStart()
{
  RLOG(0, "ROSSpinnerComponent::start()");

  if (!nh)
  {
    RLOG(0, "Creating node handle");
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  }

  if (callbackUpdatePeriod > 0.0)
  {
    // start callback update timer
    timer = nh->createWallTimer(ros::WallDuration(callbackUpdatePeriod),
                                &ROSSpinnerComponent::timerCallback, this);
  }

  // start spinner
  spinner.start();
  RLOG(0, "done ROSSpinnerComponent::start()");
}

void ROSSpinnerComponent::onStop()
{
  RLOG(0, "ROSSpinnerComponent::stop()");
  spinner.stop();
  timer.stop();
}

void ROSSpinnerComponent::timerCallback(const ros::WallTimerEvent& evt)
{
  RLOG(0, "Timer callback");
}

} /* namespace Rcs */
