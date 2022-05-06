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

#ifndef RCS_JACOCONSTRAINT_H
#define RCS_JACOCONSTRAINT_H

#include <ConstraintSet.h>

#include <Rcs_graph.h>


namespace Rcs
{

class JacoConstraint
{
public:

  JacoConstraint(const std::string& nameXYZ="XYZ World",
                 const std::string& namePolar="Polar World",
                 const std::string& nameXYZ1="XYZ Object1",
                 const std::string& namePolar1="Polar Object1",
                 const std::string& nameXYZ2="XYZ Object2",
                 const std::string& namePolar2="Polar Object2",
                 const std::string& nameFingers="Jaco1 Fingers");

  std::string getClassName() const;
  tropic::TCS_sptr get(double t_start, double duration, int object=1) const;
  tropic::TCS_sptr put(double t_start, double duration, int object=1) const;
  tropic::TCS_sptr release(double t_start, double duration, int object=1) const;
  tropic::TCS_sptr pour(double t_start, double duration, int object=1) const;
  tropic::TCS_sptr home(double t_start, double duration) const;
  tropic::TCS_sptr openFingers(double t_start, double duration) const;
  tropic::TCS_sptr closeFingers(double t_start, double duration) const;

private:

  tropic::TCS_sptr setFingers(double t_start, double duration, double angle) const;
  tropic::TCS_sptr getOrPut(double t_start, double duration, int object, bool getAndNotPut) const;

  std::string xyz, polar, xyz1, polar1, xyz2, polar2, fingers;
  double activationHorizon;
};

}   // namespace Rcs

#endif   // RCS_JACOCONSTRAINT_H
