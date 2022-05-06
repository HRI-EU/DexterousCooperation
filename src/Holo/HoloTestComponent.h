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

#ifndef RCS_HOLOTESTCOMPONENT_H
#define RCS_HOLOTESTCOMPONENT_H

#include "HoloComponent.h"



namespace Rcs
{

class HoloTestComponent : public HoloComponent
{
public:

  HoloTestComponent(EntityBase* parent, const ControllerBase* controller,
                    unsigned int port_broadcast=26794, unsigned int port_receive=26795);
  virtual ~HoloTestComponent();

private:


  // Thread callback function, should be called with networking frequency
  void callback();

  // Events
  void onStart();
  void onMotionState(bool isMoving;
  void onToggleTrajectories();
  void onForwardKinematics(const MatNd* q_des);

  // Members
  ControllerBase controller;
  bool showTrajectory;
  ViaPointTrajectoryPosition* robotLeftHandTrajectory;
  ViaPointTrajectoryPosition* robotRightHandTrajectory;
  ViaPointTrajectoryPosition* partnerLeftHandTrajectory;
  ViaPointTrajectoryPosition* partnerRightHandTrajectory;
  MatNd* robotLeftHandTrajectoryArray;
  MatNd* robotRightHandTrajectoryArray;
  MatNd* partnerLeftHandTrajectoryArray;
  MatNd* partnerRightHandTrajectoryArray;

  // Avoid copying this class
  HoloTestComponent(const HoloTestComponent&);
  HoloTestComponent& operator=(const HoloTestComponent&);
};

}

#endif   // RCS_HOLOTESTCOMPONENT_H
