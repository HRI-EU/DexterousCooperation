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

#include "DyadicPolygonObjectConstraint.h"

#include <ActivationSet.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>


using namespace tropic;

namespace Dc
{

DyadicPolygonObjectConstraint::DyadicPolygonObjectConstraint(double deltaPhi,
                                                             double deltaH) :
  tta("Phi_Box", "XYZ_R", "XYZ_L", "ABC_R", "ABC_L", "XYZ_Box", deltaPhi, deltaH),
  ttp("Partner_XYZ_R", "Partner_XYZ_L", "Partner_ABC_R", "Partner_ABC_L", deltaPhi, deltaH)
{
}

void DyadicPolygonObjectConstraint::setContactPoints(std::vector<HTr> roboContacts,
                                                     std::vector<HTr> partnerContacts)
{
  tta.setContactPoints(roboContacts);
  ttp.setContactPoints(partnerContacts);
}

void DyadicPolygonObjectConstraint::setSlideMode(bool enable)
{
  tta.setSlideMode(enable);
  ttp.setSlideMode(enable);
}

bool DyadicPolygonObjectConstraint::getSlideMode() const
{
  return tta.getSlideMode() && ttp.getSlideMode();
}

TCS_sptr DyadicPolygonObjectConstraint::createSolutionSet(std::vector<std::vector<int> > sln, double ttc) const
{
  if (sln.empty())
  {
    RLOG(1, "Solution is empty - not adding trajectory constraints");
    return nullptr;
  }

  auto moveSet = std::make_shared<ConstraintSet>();
  moveSet->setClassName("SolutionSet");

  double tReach = 0.0;
  for (size_t i = 0; i<sln.size() - 1; ++i)
  {
    RLOG(1, "Adding transition %d %d %d %d %d - %d %d %d %d %d",
         sln[i][0], sln[i][1], sln[i][2], sln[i][3], sln[i][4],
         sln[i + 1][0], sln[i + 1][1], sln[i + 1][2], sln[i + 1][3], sln[i + 1][4]);


    std::vector<int> from, to;

    from.push_back(sln[i][0]);
    from.push_back(sln[i][1]);
    from.push_back(sln[i][2]);
    from.push_back(sln[i][3]);
    from.push_back(sln[i][4]);

    to.push_back(sln[i+1][0]);
    to.push_back(sln[i+1][1]);
    to.push_back(sln[i+1][2]);
    to.push_back(sln[i+1][3]);
    to.push_back(sln[i+1][4]);

    double speedUpHuman = 1.0;

    if ((sln[i][3]!=sln[i+1][3]) || (sln[i][4]!=sln[i+1][4]))
    {
      speedUpHuman = 0.2;
    }

    auto roboSet = tta.moveFromTo(sln[i][0], sln[i][1], sln[i][2],
                                  sln[i + 1][0], sln[i + 1][1], sln[i + 1][2],
                                  tReach, speedUpHuman*ttc);

    auto humanSet = ttp.moveFromTo(sln[i][0], sln[i][3], sln[i][4],
                                   sln[i + 1][0], sln[i + 1][3], sln[i + 1][4],
                                   tReach, speedUpHuman*ttc);

    if ((!roboSet) || (!humanSet))
    {
      return nullptr;
    }

    moveSet->add(roboSet);
    moveSet->add(humanSet);

    // Add height change
    if (sln[i].size() >= 6)
    {
      int height0 = sln[i][5];
      auto hSet = tta.moveHeight(height0, tReach + ttc);

      if (!hSet)
      {
        return nullptr;
      }

      moveSet->add(hSet);
    }

    tReach += speedUpHuman*ttc;
  }

  RLOG(1, "done createSolutionSet");

  return moveSet;
}

TCS_sptr DyadicPolygonObjectConstraint::createMoveToSet(int phi, int ra, int la,
                                                        int rp, int lp, double ttc) const
{
  auto moveRobo = tta.moveTo(phi, ra, la, ttc);
  auto movePartner = ttp.moveTo(phi, rp, lp, ttc);

  if ((!moveRobo) || (!movePartner))
  {
    return nullptr;
  }

  auto moveSet = std::make_shared<ConstraintSet>();
  moveSet->add(moveRobo);
  moveSet->add(movePartner);


  return moveSet;
}

TCS_sptr DyadicPolygonObjectConstraint::createActivationSet(bool flag)
{
  auto moveSet = std::make_shared<ConstraintSet>();
  moveSet->add(tta.createActivationSet(flag));
  moveSet->add(ttp.createActivationSet(flag));

  return moveSet;
}

TCS_sptr DyadicPolygonObjectConstraint::createMoveRobotToSet(int phi, int ra, int la,
                                                             double ttc) const
{
  auto moveSet = tta.moveTo(phi, ra, la, ttc);

  if (moveSet == nullptr)
  {
    return nullptr;
  }

  return moveSet;
}

TCS_sptr DyadicPolygonObjectConstraint::driveHome(double t_home) const
{
  auto moveSet = std::make_shared<ConstraintSet>();
  const double tBlend = 3.0;
  const double t1 = tBlend;

  // Deactivate all robot tasks
  auto aSet = std::make_shared<ActivationSet>();
  aSet->addActivation(t1, false, tBlend, "XYZ_R");
  aSet->addActivation(t1, false, tBlend, "XYZ_L");
  aSet->addActivation(t1, false, tBlend, "ABC_R");
  aSet->addActivation(t1, false, tBlend, "ABC_L");

  // Deactivate all partner tasks
  aSet->addActivation(t1, false, tBlend, "Partner_XYZ_R");
  aSet->addActivation(t1, false, tBlend, "Partner_XYZ_L");
  aSet->addActivation(t1, false, tBlend, "Partner_ABC_R");
  aSet->addActivation(t1, false, tBlend, "Partner_ABC_L");

  // Activate mobile base tasks
  aSet->addActivation(t1, true, tBlend, "IME x");
  aSet->addActivation(t1, true, tBlend, "IME y");

  moveSet->add(t_home, 0.0, 0.0, 0.0, 7, "IME x 0");
  moveSet->add(t_home, 0.0, 0.0, 0.0, 7, "IME y 0");

  // Deactivate mobile base tasks
  aSet->addActivation(t_home+tBlend, false, tBlend, "IME x");
  aSet->addActivation(t_home+tBlend, false, tBlend, "IME y");

  // Stop aligning with object frontal face
  aSet->addActivation(t_home, false, tBlend, "FrontalAlignBox");

  moveSet->add(aSet);

  // Open and close fingers
  const double fngrOpenAngle = RCS_DEG2RAD(-60.0);
  moveSet->add(2.0, fngrOpenAngle, "Fingers_R 0");
  moveSet->add(2.0, fngrOpenAngle, "Fingers_R 4");
  moveSet->add(t_home, 0.0, "Fingers_R 0");
  moveSet->add(t_home, 0.0, "Fingers_R 4");

  moveSet->add(2.0, fngrOpenAngle, "Fingers_L 0");
  moveSet->add(2.0, fngrOpenAngle, "Fingers_L 4");
  moveSet->add(t_home, 0.0, "Fingers_L 0");
  moveSet->add(t_home, 0.0, "Fingers_L 4");

  return moveSet;
}


}   // namespace Rcs
