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

#ifndef RCS_NAMEDBODYFORCEDRAGGER_H
#define RCS_NAMEDBODYFORCEDRAGGER_H

#include <ForceDragger.h>


namespace Rcs
{

class NamedBodyForceDragger: public Rcs::ForceDragger
{
public:

NamedBodyForceDragger(PhysicsBase* sim) : ForceDragger(sim)
  {
  }

  virtual void update()
  {
    double f[3];
    Vec3d_sub(f, _I_mouseTip, _I_anchor);
    Vec3d_constMulSelf(f, getForceScaling()*(_leftControlPressed ? 10.0 : 1.0));

    RcsBody* simBdy = NULL;
    if (_draggedBody)
    {
      simBdy = RcsGraph_getBodyByName(physics->getGraph(), _draggedBody->name);
      NLOG(0, "Graph addy: 0x%x", physics->getGraph());
      NLOG(0, "Dragging %s to: [%f, %f, %f]", simBdy->name, f[0], f[1], f[2]);
    }
    physics->applyForce(simBdy, f, _k_anchor);

  }

  bool callback(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa)
  {
    MouseDragger::callback(ea, aa);   // calls update (see above)

    const RcsBody* bdy2 = Rcs::MouseDragger::getBodyUnderMouse(ea, aa);

    if (bdy2==NULL)
    {
      return false;
    }

    const RcsBody* bdy = RcsGraph_getBodyByName(physics->getGraph(), bdy2->name);

    switch (ea.getEventType())
    {
      /////////////////////////////////////////////////////////////////
      // Key pressed events
      /////////////////////////////////////////////////////////////////
      case (osgGA::GUIEventAdapter::KEYDOWN):
      {
        if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Left) && (bdy!=NULL))
        {
          HTr A_new;
          physics->getPhysicsTransform(&A_new, bdy);
          A_new.org[1] -= 0.01;
          physics->applyTransform(bdy, &A_new);
        }
        else if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Right) && (bdy!=NULL))
        {
          HTr A_new;
          physics->getPhysicsTransform(&A_new, bdy);
          A_new.org[1] += 0.01;
          physics->applyTransform(bdy, &A_new);
        }
        else if ((ea.getKey() == osgGA::GUIEventAdapter::KEY_Up) && (bdy!=NULL))
        {
          HTr A_new;
          physics->getPhysicsTransform(&A_new, bdy);
          A_new.org[0] += 0.01;
          physics->applyTransform(bdy, &A_new);
        }
        else if ((ea.getKey()==osgGA::GUIEventAdapter::KEY_Down) && (bdy!=NULL))
        {
          HTr A_new;
          physics->getPhysicsTransform(&A_new, bdy);
          A_new.org[0] -= 0.01;
          physics->applyTransform(bdy, &A_new);
        }

        break;
      }

      default:
        break;

    }   // switch(ea.getEventType())

    return false;
  }

};

} /// namespace Rcs

#endif   // RCS_NAMEDBODYFORCEDRAGGER_H
