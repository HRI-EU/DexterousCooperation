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

#ifndef RCS_DYADICTRAJECTORYSET_H
#define RCS_DYADICTRAJECTORYSET_H

#include <TrajectoryController.h>


namespace Rcs
{

class RelGrip;
class DyadicTrajectorySet
{
public:

  DyadicTrajectorySet(tropic::TrajectoryControllerBase* tc_, double deltaPhi);

  virtual ~DyadicTrajectorySet();
  void setContactPoints(std::vector<HTr> roboContacts,
                        std::vector<HTr> partnerContacts);
  std::vector<int> getState(int withBoxHeight=false) const;

  void setSlideMode(bool enable);
  bool getSlideMode() const;

  // \todo: No effect
  void setPartnerActive(bool enable);

  std::shared_ptr<tropic::ConstraintSet>
  createSolutionSet(std::vector<std::vector<int> > sln, double ttc) const;

  std::shared_ptr<tropic::ConstraintSet>
  createTransitionSet(int phi, int ra, int la,
                      int rp, int lp, double ttc) const;

  std::shared_ptr<tropic::ConstraintSet>
  createMoveToSet(int phi, int ra, int la,
                  int rp, int lp, double ttc) const;

  std::shared_ptr<tropic::ConstraintSet> createActivationSet(bool flag);

  std::shared_ptr<tropic::ConstraintSet>
  createMoveRobotToSet(int phi, int ra, int la, double ttc) const;

private:

  RelGrip* tta; // agent
  RelGrip* ttp; // partner

  bool controlPartner;

  /*! \brief Private assignment operator to avoid avoid memory issues when assigning
   */
  DyadicTrajectorySet& operator = (const DyadicTrajectorySet&);
  DyadicTrajectorySet(const DyadicTrajectorySet& copyFromMe);
};

}   // namespace Rcs

#endif   // RCS_DYADICTRAJECTORYSET_H
