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

#include "HoloParser.h"

#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>
#include <Rcs_quaternion.h>
#include <Rcs_typedef.h>
#include <Rcs_graphicsUtils.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>

void parseQuaternion(const std::string& quaternion, double quat[4])
{
  // parse quaternion: [x y z w] input from Unity3d
  std::vector<std::string> split = Rcs::String_split(quaternion, SEP_Vector);
  RCHECK(split.size()==4);

  RLOG(5, "[%s] [%s] [%s] [%s]", split[0].c_str(), split[1].c_str(), split[2].c_str(), quaternion.c_str());

  // Rcs convention: [w x y z]
  quat[0] = String_toDouble_l(split[3].c_str());   // w
  quat[1] = String_toDouble_l(split[0].c_str());   // x
  quat[2] = String_toDouble_l(split[1].c_str());   // y
  quat[3] = String_toDouble_l(split[2].c_str());   // z
  // RMSG("Quaternion: %f %f %f %f", quat[0], quat[1], quat[2], quat[3]);

  Quat_normalizeSelf(quat);

  double len = sqrt(Quat_dot(quat, quat));


  if (fabs(len-1.0) > 1.0e-3)
  {
    RLOG(0, "Failed to normalize quaternion");
  }

}

void parseVector(const std::string& message, double* vec, unsigned int n)
{
  std::vector<std::string> split = Rcs::String_split(message, SEP_Vector);

  if (split.size() != n)
  {
    RLOG(0, "Vector size mismatch: message '%s' contains %zu sub-strings, %u are expected", message.c_str(), split.size(), n);
    return;
  }

  for (size_t i=0; i<split.size(); ++i)
  {
    vec[i] = String_toDouble_l(split[i].c_str());
  }

  //VecNd_printComment("Parsed vector:", vec, n);
}

/// TODO: Refactor transform messages...
std::string updateTransformMsg(const std::string& name, const std::string& parent, const double pos[3], const double rot[4], std::string mirrorTarget, const double mirrorAxis[3])
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);

  writer.StartObject();
  writer.Key("_parentId");
  writer.String(parent);
  writer.Key("Id");
  writer.String(name);

  writer.Key("Position");
  writer.StartObject();
  writer.Key("x");
  writer.Double(pos[0]);
  writer.Key("y");
  writer.Double(pos[1]);
  writer.Key("z");
  writer.Double(pos[2]);
  writer.EndObject();

  writer.Key("Rotation");
  writer.StartObject();
  writer.Key("x");
  writer.Double(rot[1]);
  writer.Key("y");
  writer.Double(rot[2]);
  writer.Key("z");
  writer.Double(rot[3]);
  writer.Key("w");
  writer.Double(rot[0]);
  writer.EndObject();

  writer.Key("MirrorTarget");
  writer.String(mirrorTarget);

  if (mirrorAxis)
  {
    writer.Key("MirrorAxis");
    writer.StartObject();
    writer.Key("x");
    writer.Double(mirrorAxis[0]);
    writer.Key("y");
    writer.Double(mirrorAxis[1]);
    writer.Key("z");
    writer.Double(mirrorAxis[2]);
    writer.EndObject();
  }

  writer.EndObject();

  return MSG_Prefix + SEP_NetMsgSplit + "AddUpdateTransform" + SEP_Block + buf.GetString() + SEP_NetLastFrag;
}

std::string updateTransformMsg(const std::string& name, const std::string& parent, const HTr* A_BI)
{
  double quat[4];
  Quat_fromRotationMatrix(quat, (double(*)[3])A_BI->rot);

  return updateTransformMsg(name, parent, A_BI->org, quat);
}

std::string clearHologramMsg(std::string id, std::string topic)
{
  std::string _methodType = "ClearHologram";

  std::string _clearParameter;

  std::string _id = std::move(id);
  std::string _topic = std::move(topic);

  _clearParameter = _id + SEP_Parameter + _topic;

  return MSG_Prefix + SEP_NetMsgSplit + _methodType + SEP_Block + _clearParameter + SEP_NetLastFrag;
}

std::string toggleUIMsg(bool enableUI)
{
  std::string _methodType = "ToggleUI";
  std::string _parameter = enableUI ? "true" : "false";

  return MSG_Prefix + SEP_NetMsgSplit + _methodType + SEP_Block + _parameter + SEP_NetLastFrag;
}

std::string TTSMsg(std::string text)
{
  return MSG_Prefix + SEP_NetMsgSplit + "TextToSpeech" + SEP_Block + text + SEP_NetLastFrag;
}

std::string updateRcsBodyMsg(const RcsBody* body, const std::string& parent)
{
  std::string msg = updateTransformMsg(body->name, parent, &body->A_BI);
  return msg;
}

std::string addRcsBodyMsg(const RcsBody* body, const std::string& parent)
{
  std::string msg = updateTransformMsg(body->name, parent, &body->A_BI);

  for (unsigned int i=0; i<RcsBody_numShapes(body); ++i)
  {
    const RcsShape* shape_i = &body->shapes[i];

    // Color information from shape. In case none can be found, the shape
    // is colored white.
    // Note: sMat is stored in buffer for next lookup - don't delete it.
    unsigned int color[4] = {255, 255, 255, 255};

#if 0
    Rcs::RcsMaterialData* sMat = Rcs::getMaterial(shape_i->color);
    if (sMat)
    {
      for (int j=0; j<4; ++j)
      {
        color[j] = lround(255.0*sMat->amb[j]);
      }
    }
#else
    osg::Vec4 osgColor = Rcs::colorFromString(shape_i->color);
    for (int j=0; j<4; ++j)
    {
      color[j] = lround(255.0*osgColor[j]);
    }
#endif

    // Assemble shape name
    std::string sName = body->name+std::string("_")+std::to_string(i);

    switch (shape_i->type)
    {
      case RCSSHAPE_BOX:
        msg += updateTransformMsg(sName, std::string(body->name),
                                  &shape_i->A_CB);
        msg += Box(sName+std::string("_BOX_")+std::to_string(i),
                   body->name, color, shape_i->extents);
        break;

      case RCSSHAPE_CYLINDER:
        msg += updateTransformMsg(sName, std::string(body->name),
                                  &shape_i->A_CB);
        msg += Cylinder(sName+std::string("_CYL_")+std::to_string(i),
                        body->name, color, shape_i->extents[0],
                        shape_i->extents[2]);
        break;

      case RCSSHAPE_SPHERE:
        msg += updateTransformMsg(sName, std::string(body->name),
                                  &shape_i->A_CB);
        msg += Sphere(sName+std::string("_SPHERE_")+std::to_string(i),
                      body->name, color, shape_i->extents[0]);
        break;

      case RCSSHAPE_REFFRAME:
        msg += updateTransformMsg(sName, std::string(body->name),
                                  &shape_i->A_CB);
        msg += Axes(sName+std::string("_FRM_")+std::to_string(i),
                    body->name, shape_i->scale3d[0]);
        break;

      default:
        RLOG(1, "Unsupported shape type \"%s\"",
             RcsShape_name(shape_i->type));
    }
  }

  return msg;
}

std::string addRcsGraphMsg(const RcsGraph* graph, const std::string& parent)
{
  std::string msg;

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    msg += addRcsBodyMsg(BODY, parent);
  }

  return msg;
}

std::string updateRcsGraphMsg(const RcsGraph* graph, const std::string& parent)
{
  std::string msg;

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    msg += updateRcsBodyMsg(BODY, parent);
  }

  return msg;
}
