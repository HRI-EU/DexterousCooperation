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

#include "ActionProgressTransition.h"

#include "ActionProgressState.h"
#include "Rcs_macros.h"

namespace Rcs
{

ActionProgressTransition::ActionProgressTransition()
{

}

ActionProgressTransition::ActionProgressTransition(int id,
                                                   std::shared_ptr<ActionProgressState> start,
                                                   std::shared_ptr<ActionProgressState> stop,
                                                   std::string description)
{
  id_ = id;
  description_ = description;
  start_ = start;
  stop_ = stop;
}

ActionProgressTransition::~ActionProgressTransition()
{

}

void ActionProgressTransition::print()
{
  ActionProgressState_ptr s0 = start_.lock();
  ActionProgressState_ptr s1 = stop_.lock();

  if (s0 && s1)
  {
    printf("    ActionProgressTransition from '%s' to '%s'\n", s0->id_.c_str(), s1->id_.c_str());
  }
  else
  {
    printf("    ActionProgressTransition '%d' with invalid start or stop states!\n", id_);
  }
}

}
