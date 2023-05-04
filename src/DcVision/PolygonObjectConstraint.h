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

#ifndef DC_POLYGONOBJECTCONSTRAINT_H
#define DC_POLYGONOBJECTCONSTRAINT_H

#include <ConstraintSet.h>


namespace Dc
{

class PolygonObjectConstraint
{
public:

  PolygonObjectConstraint(const std::string& namePhi_Obj,
                          const std::string& nameXYZ_R,
                          const std::string& nameXYZ_L,
                          const std::string& nameABC_R,
                          const std::string& nameABC_L,
                          const std::string& nameXYZ_Obj,
                          double deltaPhi,
                          double deltaH);

  PolygonObjectConstraint(const std::string& nameXYZ_R,
                          const std::string& nameXYZ_L,
                          const std::string& nameABC_R,
                          const std::string& nameABC_L,
                          double deltaPhi,
                          double deltaH);

  std::string getClassName() const;
  void setContactPoints(std::vector<HTr> contacts);
  bool isPartner() const;
  void setSlideMode(bool enable);
  bool getSlideMode() const;

  tropic::TCS_sptr createActivationSet(bool active, double time=0.0) const;
  tropic::TCS_sptr moveTo(int phi, int rh, int lh, double t_goal) const;
  tropic::TCS_sptr moveFromTo(int phi0, int rh0, int lh0,
                              int phi1, int rh1, int lh1,
                              double t0, double duration) const;
  tropic::TCS_sptr moveHeight(int height, double t1) const;

private:

  double deltaPhi;
  double deltaH;
  double wristAngle;
  bool slideMode;

  std::string trajPhi;
  std::string trajPosR;
  std::string trajPosL;
  std::string trajOriR;
  std::string trajOriL;
  std::string trajPosObj;
  std::string trajFingersL;
  std::string trajFingersR;

  std::vector<HTr> contacts;
};

}   // namespace Dc

#endif   // DC_POLYGONOBJECTCONSTRAINT_H
