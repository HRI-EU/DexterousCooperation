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

#include "JacoConstraint.h"

#include <ActivationSet.h>
#include <PoseConstraint.h>
#include <PolarConstraint.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <string>


namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
JacoConstraint::JacoConstraint(const std::string& nameXYZWorld,
                               const std::string& namePolarWorld,
                               const std::string& nameXYZ1,
                               const std::string& namePolar1,
                               const std::string& nameXYZ2,
                               const std::string& namePolar2,
                               const std::string& nameFingers) :
  xyz(nameXYZWorld), polar(namePolarWorld),
  xyz1(nameXYZ1), polar1(namePolar1),
  xyz2(nameXYZ2), polar2(namePolar2),
  fingers(nameFingers), activationHorizon(1.0)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string JacoConstraint::getClassName() const
{
  return std::string("JacoConstraint");
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::put(double t_start, double duration, int object) const
{
  return getOrPut(t_start, duration, object, false);
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::get(double t_start, double duration, int object) const
{
  return getOrPut(t_start, duration, object, true);
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::getOrPut(double t_start, double duration,
                                          int object, bool getAndNotPut) const
{
  const double t_goal = t_start + duration;
  // const double fngrClosed = RCS_DEG2RAD(40.0);
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  double x = 0.1;
  double y = 0.0;
  double z = 0.0;
  double phi = 0.0;
  double theta = 0.0;

  switch (object)
  {
    case 1:
      moveSet->addActivation(t_start, true, activationHorizon, xyz1);
#if 1
      moveSet->addActivation(t_start, true, activationHorizon, polar1);
#else
      moveSet->addActivation(t_start, true, activationHorizon, polar1);
      moveSet->addActivation(t_start+0.25*duration, false, activationHorizon, polar1);
      moveSet->addActivation(t_start+0.5*duration, true, activationHorizon, polar1);

#endif
      moveSet->addActivation(t_start, false, activationHorizon, xyz2);
      moveSet->addActivation(t_start, false, activationHorizon, polar2);
      break;

    case 2:
      moveSet->addActivation(t_start, false, activationHorizon, xyz1);
      moveSet->addActivation(t_start, false, activationHorizon, polar1);
      moveSet->addActivation(t_start, true, activationHorizon, xyz2);
#if 1
      moveSet->addActivation(t_start, true, activationHorizon, polar2);
#else
      moveSet->addActivation(t_start, true, activationHorizon, polar2);
      moveSet->addActivation(t_start+0.25*duration, false, activationHorizon, polar2);
      moveSet->addActivation(t_start+0.5*duration, true, activationHorizon, polar2);

#endif
      break;

    default:
      RFATAL("Unknown object %d", object);
  }

  moveSet->addActivation(t_start, false, activationHorizon, xyz);
  moveSet->addActivation(t_start, false, activationHorizon, polar);
  moveSet->addActivation(t_start, true, activationHorizon, fingers);

  std::string taskPos = "XYZ Object" + std::to_string(object);
  std::string taskPolar = "Polar Object" + std::to_string(object);

  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal-2.0, x, y, z+0.1, taskPos, 1));
  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, x, y, z, taskPos));
  moveSet->add(std::make_shared<tropic::PolarConstraint>(t_goal, phi, theta, taskPolar));

  if (getAndNotPut)
  {
    moveSet->add(openFingers(t_start, duration));
    moveSet->add(closeFingers(t_goal, 2.0));
  }
  else
  {
    moveSet->add(closeFingers(t_start, duration));
    moveSet->add(openFingers(t_goal, 2.0));
  }


  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::release(double t_start, double duration, int object) const
{
  const double t_goal = t_start + duration;
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  double x = 0.1;
  double y = 0.0;
  double z = 0.0;
  double phi = 0.0;
  double theta = 0.0;

  switch (object)
  {
    case 1:
      moveSet->addActivation(t_start, true, activationHorizon, xyz1);
      moveSet->addActivation(t_start, true, activationHorizon, polar1);
      moveSet->addActivation(t_start, false, activationHorizon, xyz2);
      moveSet->addActivation(t_start, false, activationHorizon, polar2);
      break;

    case 2:
      moveSet->addActivation(t_start, false, activationHorizon, xyz1);
      moveSet->addActivation(t_start, false, activationHorizon, polar1);
      moveSet->addActivation(t_start, true, activationHorizon, xyz2);
      moveSet->addActivation(t_start, true, activationHorizon, polar2);
      break;

    default:
      RFATAL("Unknown object %d", object);
  }

  moveSet->addActivation(t_start, false, activationHorizon, xyz);
  moveSet->addActivation(t_start, false, activationHorizon, polar);

  moveSet->addActivation(t_goal, false, activationHorizon, fingers);
  std::string taskPos = "XYZ Object" + std::to_string(object);
  std::string taskPolar = "Polar Object" + std::to_string(object);

  RLOG_CPP(0, "taskPos = " << taskPos);
  RLOG_CPP(0, "taskPolar = " << taskPolar);

  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, x, y, z+0.1, taskPos));
  moveSet->add(std::make_shared<tropic::PolarConstraint>(t_goal, phi, theta, taskPolar));
  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.5*duration, 0.0, 0.0, 0.0, fingers));

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::pour(double t_start, double duration, int object) const
{
  const double t_half = t_start + 0.5*duration;
  const double t_goal = t_start + duration;
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  switch (object)
  {
    case 1:
      moveSet->addActivation(t_start, true, activationHorizon, xyz1);
      moveSet->addActivation(t_start, false, activationHorizon, xyz2);
      break;

    case 2:
      moveSet->addActivation(t_start, false, activationHorizon, xyz1);
      moveSet->addActivation(t_start, true, activationHorizon, xyz2);
      break;

    default:
      RFATAL("Unknown object %d", object);
  }

  moveSet->addActivation(t_start, false, activationHorizon, xyz);
  moveSet->addActivation(t_start, false, activationHorizon, polar1);
  moveSet->addActivation(t_start, false, activationHorizon, polar2);
  moveSet->addActivation(t_start, true, activationHorizon, polar);

  std::string taskPos = "XYZ Object" + std::to_string(object);
  // std::string taskPolar = "Polar Object" + std::to_string(object);

  double x = -0.15;
  double y = -0.1;
  double z = 0.0;
  double phi = 0.0;
  double theta = 0.0;

  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_half, x, y, z, taskPos));
  moveSet->add(std::make_shared<tropic::PolarConstraint>(t_half, phi, theta, polar));

  x = 0.2;
  y = 0.0;
  z = 0.05;
  phi = RCS_DEG2RAD(120.0);
  theta = M_PI_2;

  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, x, y, z, taskPos));
  moveSet->add(std::make_shared<tropic::PolarConstraint>(t_goal, phi, theta, polar));

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::home(double t_start, double duration) const
{
  const double t_goal = t_start + duration;
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  // Deactivate all relative tasks and activate task wrt. world frame
  moveSet->addActivation(t_start+activationHorizon, false, activationHorizon, xyz1);
  moveSet->addActivation(t_start+activationHorizon, false, activationHorizon, polar1);
  moveSet->addActivation(t_start+activationHorizon, false, activationHorizon, xyz2);
  moveSet->addActivation(t_start+activationHorizon, false, activationHorizon, polar2);
  moveSet->addActivation(t_start+activationHorizon, true, activationHorizon, xyz);
  moveSet->addActivation(t_start+activationHorizon, true, activationHorizon, polar);

  double x = 0.2565;
  double y = 0.2125;
  double z = 0.5561;
  double phi = RCS_DEG2RAD(11.622);
  double theta = RCS_DEG2RAD(-15.49);

  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, x, y, z, xyz));
  moveSet->add(std::make_shared<tropic::PolarConstraint>(t_goal, phi, theta, polar));

  moveSet->addActivation(t_goal, false, 0.25*duration, xyz);
  moveSet->addActivation(t_goal, false, 0.25*duration, polar);
  //moveSet->addActivation(t_start+0.5*duration, false, 0.25*duration, polar);

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::setFingers(double t_start, double duration, double angle_) const
{
  const double t_goal = t_start + duration;
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  double angle = Math_clip(angle_, 0.0, RCS_DEG2RAD(60.0));

  if (angle != angle_)
  {
    RLOG(1, "Finger angle out of range: %f not [0 ... 60] deg", RCS_DEG2RAD(angle_));
  }

  moveSet->addActivation(0.0, true, activationHorizon, fingers);
  moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, angle, angle, angle, fingers));

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::openFingers(double t_start, double duration) const
{
  return setFingers(t_start, duration, RCS_DEG2RAD(0.0));
}

/*******************************************************************************
 *
 ******************************************************************************/
tropic::TCS_sptr JacoConstraint::closeFingers(double t_start, double duration) const
{
  return setFingers(t_start, duration, RCS_DEG2RAD(60.0));// was 40
}


}   // namespace Rcs
