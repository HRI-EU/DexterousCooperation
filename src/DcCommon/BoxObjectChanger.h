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

#ifndef DC_BOXOBJECTCHANGER_H
#define DC_BOXOBJECTCHANGER_H


#include "ComponentBase.h"

#include <Rcs_joint.h>
#include <Rcs_shape.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#include <utility>
#include <vector>


namespace Dc
{
class BoxObjectChanger : public ComponentBase
{
public:

  BoxObjectChanger(EntityBase* parent) : ComponentBase(parent), objIdx(0)
  {
    getEntity()->subscribe<std::string>("ChangeObjectShape", &BoxObjectChanger::onSelectObject, this);
    getEntity()->subscribe<>("ToggleObjectShape", &BoxObjectChanger::onToggleObject, this);
  }

  virtual ~BoxObjectChanger()
  {
  }

private:

  void onSelectObject(std::string objectType)
  {
    if (objectType == "Box")
    {
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Box_real", true);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "LShape_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Cylinder_real", false);
      this->objIdx = 0;
    }
    else if (objectType == "Cylinder")
    {
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Box_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "LShape_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Cylinder_real", true);
      this->objIdx = 1;
    }
    else if (objectType == "LShape")
    {
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Box_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "LShape_real", true);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Cylinder_real", false);
      this->objIdx = 2;
    }
    else
    {
      RLOG(0, "Object type \"%s\" not supported", objectType.c_str());
      RLOG(0, "Options: Box, Cylinder, LShape");
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Box_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "LShape_real", false);
      getEntity()->publish<std::string, bool>("SetObjectActivation", "Cylinder_real", false);
      return;
    }

  }

  void onToggleObject()
  {
    this->objIdx++;
    if (this->objIdx > 2)
    {
      this->objIdx = 0;
    }

    switch (this->objIdx)
    {
      case 0:
        getEntity()->publish<std::string>("ChangeObjectShape", "Box");
        break;
      case 1:
        getEntity()->publish<std::string>("ChangeObjectShape", "Cylinder");
        break;
      case 2:
        getEntity()->publish<std::string>("ChangeObjectShape", "LShape");
        break;
      default:
        RMSG("Unknown object index %d - should be 0, 1 or 2", this->objIdx);
    }

  }

  int objIdx;
};

}

#endif   // DC_BOXOBJECTCHANGER_H
