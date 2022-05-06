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

// \todo: This file is now responsible for much more than parsing and should likely be split up into several modules.

#ifndef RCS_HOLOPARSER_H
#define RCS_HOLOPARSER_H

#include <Rcs_macros.h>
#include <Rcs_graph.h>
#include <Rcs_typedef.h>
#include <Rcs_Vec3d.h>
#include <EventSystem.h>
#include <PeriodicCallback.h>

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>

#include <string>
#include <vector>


// Colors are R-G-B-Alpha. The range for the values is between 0 and 255.
// In case the argument states three elements for a color array, alpha is
// set to 255 (solid)

// This is how a message looks like: HONDA|stuff#
const std::string SEP_Block = "&";
const std::string SEP_Parameter = "$";
const std::string SEP_Array = "~";
const std::string SEP_Vector = "\"";
const std::string SEP_NetLastFrag = "#";   // Last character of each message
const std::string SEP_NetMsgSplit = "|";
const std::string MSG_Prefix = "HONDA";

void parseQuaternion(const std::string& quaternion, double quat[4]);
void parseVector(const std::string& message, double* vec, unsigned int n);

std::string updateTransformMsg(const std::string& name,
                               const std::string& parent,
                               const double pos[3],
                               const double rot[4],
                               std::string mirrorTarget="",
                               const double mirrorAxis[3] = nullptr);

std::string updateTransformMsg(const std::string& name,
                               const std::string& parent,
                               const HTr* A_BI);

/// Creates a message to clear (destroy) one or more Holograms on connected devices.
/// \param id       Specific ID of a Hologram to clear. If the empty string is
///                 used, all Holograms matching the topic will be cleared.
/// \param topic    Topic of Hologram(s) to clear. If both topic and id are
///                 empty strings, ALL Holograms will be cleared.
/// \return         The complete network message to clear the specified Hologram(s).
std::string clearHologramMsg(std::string id, std::string topic);

/// Creates a message to make the settings UI visible or invisible on connected devices.
/// \param enableUI Whether the UI should be visible or not.
/// \return         The complete network message to set the UI visibility.
std::string toggleUIMsg(bool enableUI);

/// Creates a message to trigger a text-to-speech event on connected devices.
/// \param text The text to read.
/// \return     The complete network message containing the TTS command.
std::string TTSMsg(std::string text);

// Graph-related updates. Updating only goes through the body transforms, and
// assumes that the child hierarchy doesn't change. Adding also adds a
// transform message for the relative shape transform, and a message for the
// corresponding shape to render.
// In these functions, we only operate on the level of transforms, there's no
// consideration of joints.
std::string addRcsBodyMsg(const RcsBody* body, const std::string& parent);
std::string updateRcsBodyMsg(const RcsBody* body, const std::string& parent);
std::string addRcsGraphMsg(const RcsGraph* graph, const std::string& parent);
std::string updateRcsGraphMsg(const RcsGraph* graph, const std::string& parent);


class Hologram
{
public:
  std::string name = "Hologram"; //!< must be unique *on the Hololens*
  std::string parent = "All";    //!< name of parent Transform. "All" represents the World frame.
  std::string topic = "Default"; //!< used to filter which Holograms are visible on the device.
  unsigned int color[4] = { 255, 255, 255, 255 };   /// TODO: It could be useful to have a dedicated Color class, esp.
  bool selected = false; // maybe useless...        ///       one which provided transformations to other color spaces.
  double lifetime = 0.0; //!< <=0 : infinite, >0 : hologram will self-destruct after lifetime seconds

  Hologram() = default;

  Hologram(std::string name)
  {
    this->name = std::move(name);
  }

  Hologram(std::string name, std::string parent) : Hologram(std::move(name))
  {
    this->parent = std::move(parent);
  }

  Hologram(std::string name, std::string parent, const unsigned int color[4]) :
    Hologram(std::move(name), std::move(parent))
  {
    this->color[0] = color[0];
    this->color[1] = color[1];
    this->color[2] = color[2];
    this->color[3] = color[3];
  }

  Hologram(std::string name, std::string parent,
           std::initializer_list<unsigned int> color) :
    Hologram(std::move(name), std::move(parent))
  {
    RCHECK_MSG(color.size() == 3 || color.size() == 4,
               "Colors must contain 3 or 4 components only. Got %zd instead.",
               color.size());

    std::copy(color.begin(), color.end(), this->color);
  }

  Hologram(std::string name, std::string parent, const unsigned int color[4], double lifetime) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->lifetime = lifetime;
  }

  Hologram(std::string name, std::string parent,std::initializer_list<unsigned int> color, double lifetime) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->lifetime = lifetime;
  }

  Hologram(std::string name, std::string parent, double lifetime) : Hologram(std::move(name), std::move(parent))
  {
    this->lifetime = lifetime;
  }

  /// Used by Holograms to write out their own specific data fields for serialization.
  /// \param writer
  virtual void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const = 0;

  std::string serialize() const
  {
    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buf);

    // write out common properties for all holograms
    writer.StartObject();
    writer.Key("_parentId");
    writer.String(parent);

    writer.Key("Id");
    writer.String(name);

    writer.Key("Topic");
    writer.String(topic);

    writer.Key("LifeTime");
    writer.Double(lifetime);

    writer.Key("Selected");
    writer.Bool(selected);

    writer.Key("Color");
    writer.StartObject();
    writer.Key("r");
    writer.Uint(color[0]);
    writer.Key("g");
    writer.Uint(color[1]);
    writer.Key("b");
    writer.Uint(color[2]);
    writer.Key("a");
    writer.Uint(color[3]);
    writer.EndObject();

    // write out properties specific to this hologram
    serializeProperties(writer);

    writer.EndObject();

    return buf.GetString();
  }

  /// Set the Hologram's base color.
  /// \param red      0-255
  /// \param green    0-255
  /// \param blue     0-255
  /// \param alpha    0-255
  virtual void setColor(unsigned int red, unsigned int green, unsigned int blue, unsigned int alpha = 255)
  {
    color[0] = red;
    color[1] = green;
    color[2] = blue;
    color[3] = alpha;
  }

  /// Set the hologram's base color.
  /// \param Color RGBA order, each channel 0-255
  void setColor(const unsigned int Color[4])
  {
    setColor(Color[0], Color[1], Color[2], Color[3]);
  }

  /// Set the Hologram's base color.
  /// \param Color Vector of length 3 (RGB) or 4 (RGBA), with values between 0 and 255.
  void setColor(const std::vector<unsigned int>& Color)
  {
    RCHECK(Color.size() >= 3);

    if (Color.size() == 3)
    {
      setColor(Color[0], Color[1], Color[2]);
    }
    else
    {
      setColor(Color[0], Color[1], Color[2], Color[3]);
    }
  }

  /// \return The type of this hologram, as defined by the HologramManager on the Unity side
  virtual std::string getType() const = 0;

  /// \return The event used by HoloComponent to update this hologram
  static std::string getEventName()
  {
    return "";
  };

  /// Convenience method for publishing holograms. Automatically uses a correct default event matching
  /// HoloComponents event API.
  /// TODO: There is likely a better way to do this...
  /// \tparam H       Hologram type to publish.
  /// \param hologram Concrete hologram.
  /// \param es       ES::EventSystem to publish hologram to.
  template<typename H>
  static void publish(H hologram, ES::EventSystem* es)
  {
    es->publish(H::getEventName(), hologram);
  }

  virtual void publish(ES::EventSystem* es) const = 0;

  /// Direct conversion to a network message string.
  /// \return
  operator std::string() const
  {
    return MSG_Prefix + SEP_NetMsgSplit + "AddUpdateHologram" + SEP_Block + this->getType() + SEP_Block
           + this->serialize() + SEP_NetLastFrag;
  }

  /// Linear interpolation between two colors.
  /// Interpolates from color0 to color1 ( 0 <= t <= 1 )
  /// TODO: Do it right https://www.alanzucconi.com/2016/01/06/colour-interpolation/
  static void Colerp(const unsigned int color0[4], const unsigned int color1[4], const double t,
                     unsigned int result[4])
  {
    if (t <= 0)
    {
      for (size_t i = 0; i < 4; i++)
      {
        result[i] = color0[i];
      }
    }
    else if (t >= 1)
    {
      for (size_t i = 0; i < 4; i++)
      {
        result[i] = color1[i];
      }
    }
    else
    {
      double c0[3], c1[3], r[3];
      for (size_t i = 0; i < 3; i++)
      {
        c0[i] = (double)color0[i];
        c1[i] = (double)color1[i];
      }
      Vec3d_lerp(r, c0, c1, t);
      for (size_t i = 0; i < 3; i++)
      {
        result[i] = (unsigned int)r[i];
      }
      result[3] = (unsigned  int)((1.0-t)*(double)color0[3] + t*(double)color1[3]);
    }
  }

  /// Linear interpolation between two colors.
  /// Interpolates from color0 to color1 ( 0 <= t <= 1 )
  /// Input colors should have the same length, either RGB or RGBA
  /// TODO: Do it right https://www.alanzucconi.com/2016/01/06/colour-interpolation/
  static std::vector<unsigned int> Colerp(const std::vector<unsigned int>& color0,
                                          const std::vector<unsigned int>& color1,
                                          const double t)
  {
    RCHECK((color0.size() == 3 || color0.size() == 4) && (color1.size() == 3 || color1.size() == 4));

    std::vector<unsigned int> result;

    if (t <= 0)
    {
      for (auto& c : color0)
      {
        result.push_back((unsigned int)c);
      }
      if (color0.size() < 4)
      {
        result.push_back(255);
      }
    }
    else if (t >= 1)
    {
      for (auto& c : color1)
      {
        result.push_back((unsigned int)c);
      }
      if (color1.size() < 4)
      {
        result.push_back(255);
      }
    }
    else
    {
      double c0[4], c1[4], r[4];
      std::copy(color0.begin(), color0.end(), c0);
      std::copy(color1.begin(), color1.end(), c1);
      Vec3d_lerp(r, c0, c1, t);

      for (size_t i = 0; i < 3; i++)
      {
        result.push_back((unsigned int)r[i]);
      }

      if (color0.size() == 4 && color1.size() == 4)
      {
        result.push_back((unsigned  int)((1.0-t)*(double)c0[3] + t*(double)c1[3]));
      }
      else
      {
        result.push_back(255);
      }
    }

    return result;
  }

  /// Linear interpolation between two colors.
  /// Interpolates from color0 to color1 ( 0 <= t <= 1 )
  static std::vector<unsigned int> Colerp(std::initializer_list<double> color0,
                                          std::initializer_list<double> color1, double t)
  {
    RCHECK((color0.size() == 3 || color0.size() == 4) && (color1.size() == 3 || color1.size() == 4));

    std::vector<unsigned int> result;

    if (t <= 0)
    {
      for (auto& c : color0)
      {
        result.push_back((unsigned int)c);
      }
      if (color0.size() < 4)
      {
        result.push_back(255);
      }
    }
    else if (t >= 1)
    {
      for (auto& c : color1)
      {
        result.push_back((unsigned int)c);
      }
      if (color1.size() < 4)
      {
        result.push_back(255);
      }
    }
    else
    {
      double c0[4], c1[4], r[4];
      std::copy(color0.begin(), color0.end(), c0);
      std::copy(color1.begin(), color1.end(), c1);
      Vec3d_lerp(r, c0, c1, t);

      for (size_t i = 0; i < 3; i++)
      {
        result.push_back((unsigned int)r[i]);
      }

      if (color0.size() == 4 && color1.size() == 4)
      {
        result.push_back((unsigned  int)((1.0-t)*(double)c0[3] + t*(double)c1[3]));
      }
      else
      {
        result.push_back(255);
      }
    }

    return result;
  }
};

/// Represents a set of (XYZ) coordinate axes
class Axes : public Hologram
{
public:
  //! Length, in meters, of each axis line.
  double scale = 0.2;

  using Hologram::Hologram;
  Axes() = default;

  ///
  /// \param name     Unique name
  /// \param parent   Parent Transform
  /// \param scale    Length, in meters, of each axis line
  Axes(std::string name, std::string parent, double scale) : Hologram(std::move(name), std::move(parent))
  {
    this->scale = scale;
  }

  std::string getType() const override
  {
    return "Axes";
  }

  static std::string getEventName()
  {
    return "HoloUpdateAxes";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Scale");
    writer.Double(scale);
  }

};

/// Represents an arbitrarily-sized box
class Box : public Hologram
{
public:
  //! X, Y, Z size, in meters
  double extents[3] = {0.2, 0.2, 0.2};

  using Hologram::Hologram;
  Box() = default;
  Box(std::string name, std::string parent, unsigned const int color[4], const double extents[3]) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->extents[0] = extents[0];
    this->extents[1] = extents[1];
    this->extents[2] = extents[2];
  }
  Box(std::string name, std::string parent, std::initializer_list<unsigned int> color, const double extents[3]) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->extents[0] = extents[0];
    this->extents[1] = extents[1];
    this->extents[2] = extents[2];
  }

  std::string getType() const override
  {
    return "Poly3DCube";
  }

  static std::string getEventName()
  {
    return "HoloUpdateBox";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Extents");
    writer.StartObject();
    writer.Key("x");
    writer.Double(extents[0]);
    writer.Key("y");
    writer.Double(extents[1]);
    writer.Key("z");
    writer.Double(extents[2]);
    writer.EndObject();
  }

  /// Set the Hologram's extents
  /// \param newExtents XYZ order, in meters
  void setExtents(const double newExtents[3])
  {
    extents[0] = newExtents[0];
    extents[1] = newExtents[1];
    extents[2] = newExtents[2];
  }

  /// Set the Hologram's extents
  /// \param x (in meters)
  /// \param y (in meters)
  /// \param z (in meters)
  void setExtents(double x, double y, double z)
  {
    extents[0] = x;
    extents[1] = y;
    extents[2] = z;
  }
};

/// A solid Box which occludes any Holograms inside or behind it.
/// Give it a solid black color to make it invisible (while preserving the occlusion effect).
class BoxOccluder : public Box
{
public:
  using Box::Box;
  BoxOccluder() = default;
  BoxOccluder(std::string name, std::string parent, unsigned const int color[4], const double extents[3]) :
    Box(name, parent, color, extents)
  {
  }
  BoxOccluder(std::string name, std::string parent,
              std::initializer_list<unsigned int> color,
              const double extents[3]) : Box(name, parent, color, extents)
  {
  }

  std::string getType() const override
  {
    return "Poly3DCubeOccluder";
  }

  static std::string getEventName()
  {
    return "HoloUpdateBoxOccluder";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }
};

/// A simple Sphere
class Sphere : public Hologram
{
public:
  //! Sphere's radius, in meters
  double radius = 0.25;

  using Hologram::Hologram;
  Sphere() = default;
  Sphere(std::string name, std::string parent, const unsigned int color[4], double radius) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->radius = radius;
  }
  Sphere(std::string name, std::string parent, std::initializer_list<unsigned int> color, double radius) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->radius = radius;
  }

  std::string getType() const override
  {
    return "Poly3DSphere";
  }

  static std::string getEventName()
  {
    return "HoloUpdateSphere";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Radius");
    writer.Double(radius);
  }

};

/// A solid sphere which completely occludes objects inside or behind it.
/// Give it a solid black color to make it invisible (while preserving the occlusion effect).
class SphereOccluder : public Sphere
{
public:
  using Sphere::Sphere;
  SphereOccluder() = default;
  SphereOccluder(std::string name, std::string parent, const unsigned int color[4], double radius) :
    Sphere(name, parent, color, radius)
  {
  }
  SphereOccluder(std::string name, std::string parent, std::initializer_list<unsigned int> color, double radius) :
    Sphere(name, parent, color, radius)
  {
  }

  std::string getType() const override
  {
    return "Poly3DSphereOccluder";
  }

  static std::string getEventName()
  {
    return "HoloUpdateSphereOccluder";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }
};

class Cylinder : public Hologram
{
public:
  double radius = 0.2;
  double length = 0.4;

  using Hologram::Hologram;
  Cylinder() = default;
  Cylinder(std::string name, std::string parent, const unsigned int color[4], double radius, double length) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->radius = radius;
    this->length = length;
  }
  Cylinder(std::string name, std::string parent, std::initializer_list<unsigned int> color, double radius,
           double length) :
    Hologram(std::move(name), std::move(parent), color)
  {
    this->radius = radius;
    this->length = length;
  }

  std::string getType() const override
  {
    return "Poly3DCylinder";
  }

  static std::string getEventName()
  {
    return "HoloUpdateCylinder";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Radius");
    writer.Double(radius);
    writer.Key("Length");
    writer.Double(length);
  }

};

/// A solid Cylinder which completely occludes objects inside or behind it.
/// Give it a solid black color to make it invisible (while preserving the occlusion effect).
class CylinderOccluder : public Cylinder
{
public:
  using Cylinder::Cylinder;
  CylinderOccluder() = default;
  CylinderOccluder(std::string name, std::string parent, const unsigned int color[4], double radius, double length) :
    Cylinder(name, parent, color, radius, length)
  {
  }
  CylinderOccluder(std::string name, std::string parent,
                   std::initializer_list<unsigned int> color, double radius, double length) :
    Cylinder(name, parent, color, radius, length)
  {
  }

  std::string getType() const override
  {
    return "Poly3DCylinderOccluder";
  }

  static std::string getEventName()
  {
    return "HoloUpdateCylinderOccluder";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }
};

/// A flat, solid, arrow which points from its parent to a specified target Transform.
/// If the target moves, the arrow will automatically update to follow it.
class ArrowPlain : public Hologram
{
public:
  //! The transform to track
  std::string targetTransform;
  //! If proportional==true, length is normalized to allow 1.0 to be the exact distance between parent and target.
  //! If proportional==false, length is in meters.
  double length = 1.0;

  bool proportional = true;

  using Hologram::Hologram;
  ArrowPlain() = default;

  std::string getType() const override
  {
    return "ArrowPlain";
  }

  static std::string getEventName()
  {
    return "HoloUpdateArrowPlain";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("_endId");
    writer.String(targetTransform);
    writer.Key("Length");
    writer.Double(length);
    writer.Key("IsProportional");
    writer.Bool(proportional);

  }

};

/// Same as an ArrowPlain, but contains an internal progress bar which can be filled.
class ArrowProgress : public Hologram
{
public:
  //! The transform to track
  std::string targetTransform;
  //! If proportional==true, length is normalized to allow 1.0 to be the exact distance between parent and target.
  //! If proportional==false, length is in meters.
  double length = 1.0;
  bool proportional = true;
  //! Value between 0 and 1, representing how "filled" the arrow should be.
  double progress = 1.0;

  using Hologram::Hologram;
  ArrowProgress() = default;

  std::string getType() const override
  {
    return "ArrowProgress";
  }

  static std::string getEventName()
  {
    return "HoloUpdateArrowProgress";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("_end");
    writer.String(targetTransform);
    writer.Key("Length");
    writer.Double(length);
    writer.Key("IsProportional");
    writer.Bool(proportional);
    writer.Key("Progress");
    writer.Double(progress);
  }

};

/// A plain arrow which points from its parent transform into some direction.
class ArrowDirection : public Hologram
{
public:
  //! Direction to point in.
  double tip[3] = {1.0, 1.0, 1.0};
  //! If proportional==true, length is normalized to allow 1.0 to be the exact distance between parent and tip.
  //! If proportional==false, length is in meters.
  double length = 1.0;
  bool proportional = true;
  //! If true, tip is in world coordinates.
  //! If false, tip is in the parent Transform's frame.
  bool inWorld = false;

  using Hologram::Hologram;
  ArrowDirection() = default;

  std::string getType() const override
  {
    return "ArrowDirection";
  }

  static std::string getEventName()
  {
    return "HoloUpdateArrowDirection";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Tip");
    writer.StartObject();
    writer.Key("x");
    writer.Double(tip[0]);
    writer.Key("y");
    writer.Double(tip[1]);
    writer.Key("z");
    writer.Double(tip[2]);
    writer.EndObject();
    writer.Key("Length");
    writer.Double(length);
    writer.Key("IsProportional");
    writer.Bool(proportional);
    writer.Key("IsWorldSpace");
    writer.Bool(inWorld);
  }

};

// Text can be formatted with inline xml-style markup, see:
// https://docs.unity3d.com/Manual/StyledText.html
// parent: Where label is displayed
// targetTransform: Where line is pointing at
class Label : public Hologram
{
public:
  std::string text = "TextLabel";
  std::string targetTransform;

  using Hologram::Hologram;
  Label() = default;

  std::string getType() const override
  {
    return "Poly2DLabel";
  }

  static std::string getEventName()
  {
    return "HoloUpdateLabel";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Text");
    writer.String(text);
    writer.Key("_targetId");
    writer.String(targetTransform);
  }

};

/// A flat 2D image, which always faces the user.
class Sprite : public Hologram
{
public:
  //! Specific image to use (see 'Resources/sprites' in Unity project)
  std::string spriteName = "Sprite";
  //! Transform to point a solid white line at (set to parent if no line is desired)
  std::string targetTransform;
  double scale = 1.0;

  using Hologram::Hologram;
  Sprite() = default;

  std::string getType() const override
  {
    return "Poly2DSprite";
  }

  static std::string getEventName()
  {
    return "HoloUpdateSprite";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Name");
    writer.String(spriteName);
    writer.Key("_targetId");
    writer.String(targetTransform.empty() ? parent : targetTransform);
    writer.Key("Scale");
    writer.Double(scale);
  }

};

/// An image which also represents a progress bar or similar state.
class ProgressSprite : public Hologram
{
public:
  //! Specific image to use (see 'Resources/sprites' in Unity project)
  std::string spriteName = "progress/circle01";
  //! Value between 0 and 1, representing how "full" the progress bar should be
  double progress = 1.0;

  using Hologram::Hologram;
  ProgressSprite() = default;

  std::string getType() const override
  {
    return "ProgressSprite";
  }

  static std::string getEventName()
  {
    return "HoloUpdateProgressSprite";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("SpriteName");
    writer.String(spriteName);
    writer.Key("Progress");
    writer.Double(progress);
  }
};

/// A marker which drops a vertical line down to the floor below it.
class DropMarker : public Hologram
{
public:
  std::string getType() const override
  {
    return "Poly2DDropMarker";
  }

  static std::string getEventName()
  {
    return "HoloUpdateDropMarker";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  using Hologram::Hologram;
  DropMarker() = default;

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override {}
};

/// A 3D model with no joints.
class StaticModel : public Hologram
{
public:
  //! Name of the specific model to use (e.g. "Fox" or "Hololens")
  std::string model;
  double scale = 1.0;

  using Hologram::Hologram;
  StaticModel() = default;

  StaticModel(std::string modelName, double scale=1.0) : StaticModel()
  {
    this->model = std::move(modelName);
    this->scale = scale;
  }

  std::string getType() const override
  {
    return "Poly3DStaticModel";
  }

  static std::string getEventName()
  {
    return "HoloUpdateStaticModel";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("ModelName");
    writer.String(model);
    writer.Key("Scale");
    writer.Double(scale);
  }

};

/// A 3D model which contains configurable joints
class PosedModel : public Hologram
{
public:
  std::string model;    //!< main model to load
  std::string subModel; //!< specific component of main model to pose [DEPRECATED]
  std::vector<double> jointAngles; //!< angles, as DEGREES

  using Hologram::Hologram;
  PosedModel() = default;

  // Sets the complete pose of the specified sub-model using the joint angles found in q
  //
  // Current models available: "HondaRobot", "SDH", "KukaLBR"
  //    Current sub-models available (for "HondaRobot"; deprecated): "LeftArm", "RightArm", "LeftHand", "RightHand"
  //
  // Example usage:
  //    setPose("SDH", "RightHand", controller.getGraph()->q,
  //            RcsGraph_getJointByName(graph, "knuck3-base_R")->jointIndex, 8);
  //    setPose("HondaRobot", "LeftArm", graph->q, RcsGraph_getJointByName(graph, "lbr_joint_1_L")->jointIndex, 7)
  //    setPose("HondaRobot", "RightArm", graph->q, RcsGraph_getJointByName(graph, "lbr_joint_1_R")->jointIndex, 7)
  //    setPose("HondaRobot", "LeftHand", graph->q, RcsGraph_getJointByName(graph, "knuck3-base_L")->jointIndex, 8)
  //    setPose("HondaRobot", "RightHand", graph->q, RcsGraph_getJointByName(graph, "knuck3-base_R")->jointIndex, 8)
  void setPose(std::string modelName, const MatNd* q, size_t startIndex, size_t numJoints)
  {
    jointAngles.clear();

    this->model = std::move(modelName);

    for (size_t i = 0; i < numJoints; i++)
    {
      jointAngles.emplace_back(RCS_RAD2DEG(q->ele[startIndex+i]));
    }
  }

  void setPose(std::string modelName, const RcsGraph* graph, std::string rootJointName, size_t numJoints)
  {
    auto jnt = RcsGraph_getJointByName(graph, rootJointName.c_str());

    if (jnt != nullptr)
    {
      setPose(std::move(modelName), graph->q, jnt->jointIndex, numJoints);
    }
    else
    {
      RWARNING(0, "Joint '%s' could not be found in the given graph!", rootJointName.c_str());
    }
  }

  void setPose(std::string modelName, const RcsGraph* graph, size_t rootJointIndex, size_t numJoints)
  {
    setPose(std::move(modelName), graph->q, rootJointIndex, numJoints);
  }

  std::string getType() const override
  {
    return "Poly3DPoseModel";
  }

  static std::string getEventName()
  {
    return "HoloUpdatePosedModel";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("ModelName");
    writer.String(model);

    writer.Key("SubModelName");
    writer.String(subModel);

    writer.Key("Properties");
    writer.StartArray();
    for (auto& j : jointAngles)
    {
      writer.Double(j);
    }
    writer.EndArray();
  }

};

// Common functionality shared by different Trajectory types
class TrajectoryBase : public Hologram
{
public:
  enum class OrderingDirection
  {
    Forwards,
    Backwards
  };

  std::vector<std::vector<double>> points; //!< [ (x,y,z), (x,y,z), ... ]
  std::vector<std::vector<unsigned int>> colors;  //!< [ (r,g,b,a), (r,g,b,a), ... ]  length must be <= 8
  double lineWidth = 0.8;
  double animationSpeed = 0; //!< speed at which to cycle colors along trajectory
  bool alternating = false; //!< unused TODO: Remove from here AND from the Unity project

  using Hologram::Hologram;
  TrajectoryBase() = default;

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Points");
    writer.StartArray();
    for (auto& p : points)
    {
      writer.StartObject();
      writer.Key("x");
      writer.Double(p[0]);
      writer.Key("y");
      writer.Double(p[1]);
      writer.Key("z");
      writer.Double(p[2]);
      writer.EndObject();
    }
    writer.EndArray();

    writer.Key("AlternatingColors");
    writer.StartArray();
    for (auto& c : colors)
    {
      writer.StartObject();
      writer.Key("r");
      writer.Uint(c[0]);
      writer.Key("g");
      writer.Uint(c[1]);
      writer.Key("b");
      writer.Uint(c[2]);
      writer.Key("a");
      writer.Uint(c[3]);
      writer.EndObject();
    }
    writer.EndArray();

    writer.Key("AnimationSpeed");
    writer.Double(animationSpeed);

    writer.Key("IsAlternating");
    writer.Bool(alternating);

    writer.Key("LineWidth");
    writer.Double(lineWidth);
  }

  /// Set the points on the trajectory. Trajectory length==m, n must be 3 (xyz).
  /// \param trajectory
  void setPoints(MatNd* trajectory)
  {
    points.clear();

    for (size_t i = 0; i < trajectory->m; i++)
    {

      double p_obj[3];
      Vec3d_setZero(p_obj);
      for (size_t j = 0; j < trajectory->n; j++)
      {
        p_obj[j] = MatNd_get(trajectory, i, j);
      }

      points.emplace_back();
      for (size_t j = 0; j < trajectory->n; j++)
      {
        points[i].push_back(p_obj[j]);
      }
    }
  }

  /// Set the points on the trajectory, optionally in reverse order given
  /// \param trajectory
  /// \param direction Whether trajectory should be read forwards or backwards
  void setPoints(std::vector<HTr> trajectory, OrderingDirection direction = OrderingDirection::Forwards)
  {
    points.clear();

    if (direction == OrderingDirection::Forwards)
    {
      for (size_t i = 0; i < trajectory.size(); i++)
      {
        points.push_back({trajectory[i].org[0], trajectory[i].org[1], trajectory[i].org[2]});
      }
    }
    else
    {
      for (size_t i = trajectory.size()-1; i < trajectory.size(); i--)
      {
        points.push_back({trajectory[i].org[0], trajectory[i].org[1], trajectory[i].org[2]});
      }
    }
  }

  void setPoints(std::vector<std::vector<double>>&& trajectory)
  {
    points = trajectory;
  }

  /// Assign gradient color keys to this trajectory. Colors should be ordered [r, g, b, a]
  /// NOTE: Only the first 8 entries of newColors will be used. Additional entries will be ignored.
  void setColors(std::vector<std::vector<unsigned int>> newColors)
  {
    RCHECK_MSG(newColors.size() <= 8, "Cannot assign more than 8 color keys to a trajectory!");

    colors.clear();
    for (size_t i = 0; i < newColors.size() && i < 8; i++) // ensure we never set more than 8 color keys
    {
      colors.push_back(newColors[i]);
    }
  }

  /// Assign a solid color to this trajectory
  void setColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255)
  {
    colors.clear();
    colors.push_back({r, g, b, a});
  }

  /// Assign a solid color to this trajectory
  void setColors(std::initializer_list<std::initializer_list<unsigned int>> newColors)
  {
    RCHECK_MSG(newColors.size() <= 8, "Cannot assign more than 8 color keys to a trajectory!");

    colors.clear();
    for (auto c : newColors)
    {
      colors.emplace_back(c);
    }
  }
};

/// A trajectory of small 2D images which will always face the user
class TrajectoryPoints : public TrajectoryBase
{
public:
  //! Specific image to use for points
  std::string pointType = "crosshair002"; //!< currently available: 'crosshair001' .. 'crosshair200'
  using TrajectoryBase::TrajectoryBase;
  TrajectoryPoints() = default;

  std::string getType() const override
  {
    return "TrajectoryPoints";
  }

  static std::string getEventName()
  {
    return "HoloUpdateTrajectoryPoints";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    TrajectoryBase::serializeProperties(writer);

    writer.Key("PointType");
    writer.String(pointType);
  }
};

/// A trajectory of 3D arrows, which will point in the direction of the line they form
class TrajectoryArrows : public TrajectoryBase
{
public:
  using TrajectoryBase::TrajectoryBase;
  TrajectoryArrows() = default;

  std::string getType() const override
  {
    return "TrajectoryArrows";
  }

  static std::string getEventName()
  {
    return "HoloUpdateTrajectoryArrows";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }
};

/// A solid line connecting all points in a trajectory
class TrajectoryLines : public TrajectoryBase
{
public:
  using TrajectoryBase::TrajectoryBase;
  TrajectoryLines() = default;

  std::string getType() const override
  {
    return "TrajectoryLines";
  }

  static std::string getEventName()
  {
    return "HoloUpdateTrajectoryLines";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }
};

/// A 2D polygon
class Poly2D : public Hologram
{
public:
  std::vector<std::vector<double>> points;    //!< [ (x,y), (x,y), ... ]  Must not contain duplicate points
  std::vector<double> direction = {1, 0, 0};  //!< [ x, y, z ]            vector of surface normal -- currently
  //!<                        IGNORED, transform parent to align local XY
  //!<                        plane with your desired polygon instead

  using Hologram::Hologram;
  Poly2D() = default;

  std::string getType() const override
  {
    return "Poly2DGeometry";
  }

  static std::string getEventName()
  {
    return "HoloUpdatePoly2D";
  }

  void publish(ES::EventSystem* es) const override
  {
    Hologram::publish(*this, es);
  }

  void serializeProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer) const override
  {
    writer.Key("Points");
    writer.StartArray();
    for (auto& p : points)
    {
      writer.StartObject();
      writer.Key("x");
      writer.Double(p[0]);
      writer.Key("y");
      writer.Double(p[1]);
      writer.EndObject();
    }
    writer.EndArray();

    writer.Key("Direction");
    writer.StartObject();
    writer.Key("x");
    writer.Double(direction[0]);
    writer.Key("y");
    writer.Double(direction[1]);
    writer.Key("z");
    writer.Double(direction[2]);
    writer.EndObject();
  }

  /// Set the points used to construct the polygon. Points form the outer perimeter of the polygon and must be in
  ///     clockwise or counter-clockwise order.
  /// Adjacent duplicate points will be discarded.
  /// \param newPoints Vector containing points as [ [x1, y1], [x1, y2], ... ].
  ///                  Each point may contain more than just x,y coords, but only x,y will be used.
  void setPoints(const std::vector<std::vector<double>>& newPoints)
  {
    points.clear();
    for (size_t i = 0; i < newPoints.size(); i++)
    {
      RCHECK(newPoints[i].size() >= 2);
      if (i > 0 && newPoints[i] == newPoints[i-1])
      {
        continue;
      }

      points.push_back({newPoints[i][0], newPoints[i][1]});
    }
  }

  /// Set the points used to construct the polygon. Points form the outer perimeter of the polygon and must be in
  ///     clockwise or counter-clockwise order.
  /// Adjacent duplicate points will be discarded.
  /// \param newPoints    All points are assumed to lie on the XY plane (Z coords will be discarded)
  void setPoints(const std::vector<HTr>& newPoints)
  {
    points.clear();
    for (size_t i = 0; i < newPoints.size(); i++)
    {
      if (i > 0 &&
          newPoints[i].org[0] == newPoints[i-1].org[0] &&
          newPoints[i].org[1] == newPoints[i-1].org[1])
      {
        continue;
      }

      points.push_back({newPoints[i].org[0], newPoints[i].org[1]});
    }
  }

  /// Sets the normal (or facing direction) of the resulting polygon. Expected to be in LOCAL coordinates.
  /// \param x
  /// \param y
  /// \param z
  void setNormal(double x, double y, double z)
  {
    direction[0] = x;
    direction[1] = y;
    direction[2] = z;
  }

  /// Sets the normal (or facing direction) of the resulting polygon. Expected to be in LOCAL coordinates.
  /// \param normal Normal specified as [x, y, z]
  void setNormal(const std::vector<double> normal)
  {
    RCHECK(normal.size() >= 3);
    direction[0] = normal[0];
    direction[1] = normal[1];
    direction[2] = normal[2];
  }

};

/// Abstract base class to be used in conjuntion with Tween<TweenPublisher> for some primitive animation effects.
/// Provides the concrete implementation of shape creation and updates.
/// TODO: Come up with a better class name?
class TweenPublisher
{
public:
  /// Called each update tick of the managing Tween object.
  /// \param es       The ES::EventSystem any updates should be published to
  /// \param time     Time elapsed since Tween start (0 <= time <= duration)
  /// \param duration Total duration of the Tween (can be expected to be >= 0)
  virtual void publish(ES::EventSystem* es, double time, double duration) = 0;

  /// Called when the managing Tween object is destroyed (i.e. no future updates will be made). Should be used to
  /// perform any necessary cleanup actions.
  /// \param es       If this object needs to notify other components of its destruction (e.g. if it had created some
  ///                 Holograms which now need to be destroyed) it should use this EventSystem.
  virtual void clear(ES::EventSystem* es) = 0;

};

/// Interpolates the color of an object over time
/// \tparam H   Hologram type
template <typename H>
class ColorTweener : public TweenPublisher
{
public:
  ///
  /// \param hologram     Hologram to animate
  /// \param colorKeys    List of normalized (0..1) time points and corresponding colors. Colors between time
  ///                     points will be linearly interpolated between neighbors.
  ///                     Example: \code{.cpp}
  ///                        { {0.0, {255, 0, 0},  //  Set color to red at start
  ///                          {0.5, {0, 255, 0},  //  Interpolate to solid green at 50% of duration
  ///                          {1.0, {0, 0, 255} } //  And from green, interpolate to solid blue at the end of the
  ///                                                  animation
  ///                           \endcode
  /// \param destroyOnEnd Whether to remove the hologram from visualization after the animation completes (set
  ///                     to false, for example, if you use this to fade an object into existence).
  ColorTweener(H hologram, std::vector<std::pair<double, std::vector<unsigned int>>> colorKeys, bool destroyOnEnd=true)
    : hologram(hologram), keys(std::move(colorKeys)), destroyOnEnd(destroyOnEnd)
  {
  }

  void publish(ES::EventSystem* es, double time, double duration) override
  {
    // get our normalized time wrt total duration (0..duration -> 0..1)
    double progress = time / duration;
    if (progress > keys[nextKey].first)
    {
      nextKey++;
    }
    if (nextKey >= keys.size())
    {
      return;
    }

    // get our normalized time wrt current key pair (t1..t2 -> 0..1)
    double nextKeyTime = keys[nextKey].first;
    double prevKeyTime = nextKey > 0 ? keys[nextKey-1].first : 0;
    double t = (progress - prevKeyTime) / (nextKeyTime-prevKeyTime);

    // interpolate color keys
    hologram.setColor(Hologram::Colerp(keys[nextKey-1].second, keys[nextKey].second, t));

    // publish result
    hologram.publish(es);
  }

  void clear(ES::EventSystem* es) override
  {
    if (destroyOnEnd)
    {
      es->publish("HoloClear", hologram.name, hologram.topic);
    }
  }

private:
  H hologram;
  std::vector<std::pair<double, std::vector<unsigned int>>> keys;
  bool destroyOnEnd;

  size_t nextKey = 1;
};

/// Represents an axis-aligned ring which animates from a given starting radius to a final radius.
class CircleTweener : public TweenPublisher
{
public:
  enum class Axis
  {
    XY,
    YZ,
    ZX
  };

  CircleTweener(const std::string& name, const std::string& parent, double startRadius, double endRadius, Axis axis,
                size_t numpoints=16)
    : r0(startRadius), r1(endRadius), axis(axis)
  {
    traj.name = name;
    traj.parent = parent;

    // generate a circle of points aligned with the specified axis
    for (size_t i = 0; i <= numpoints; i++)
    {
      double c1 = startRadius * cos(2.0* M_PI * (double)i / (double)(numpoints));
      double c2 = startRadius * sin(2.0* M_PI * (double)i / (double)(numpoints));

      if (axis == Axis::XY)
        traj.points.push_back({c1, c2, 0});
      else if (axis == Axis::YZ)
        traj.points.push_back({0, c1, c2});
      else if (axis == Axis::ZX)
        traj.points.push_back({c2, 0, c1});
    }
  }

  void publish(ES::EventSystem* es, double time, double duration) override
  {
    double t = time / duration;
    double newRadius = (1.0 - t) * r0 + t * r1;
    for (size_t i = 0; i < traj.points.size(); i++)
    {
      double c1 = newRadius * cos(2.0 * M_PI * (double)i / (double)(traj.points.size()-1));
      double c2 = newRadius * sin(2.0 * M_PI * (double)i / (double)(traj.points.size()-1));

      if (axis == Axis::XY)
        traj.points[i] = {c1, c2, 0};
      else if (axis == Axis::YZ)
        traj.points[i] = {0, c1, c2};
      else if (axis == Axis::ZX)
        traj.points[i] = {c2, 0, c1};
    }

    traj.publish(es);
  }

  void clear(ES::EventSystem* es) override
  {
    es->publish("HoloClear", traj.name, traj.topic);
    es->publish("HoloClear", traj.name, traj.topic);
  }

private:
  TrajectoryPoints traj;
  double r0, r1;
  Axis axis;
};

/// Represents a sphere which animates from one radius to another.
class SphereTweener : public TweenPublisher
{
public:
  SphereTweener(const std::string& name, const std::string& parent, double startRadius, double endRadius,
                const std::vector<unsigned int> startColor, const std::vector<unsigned int> endColor)
    : r0(startRadius), r1(endRadius), c0(std::move(startColor)), c1(std::move(endColor))
  {
    sphere.name = name;
    sphere.parent = parent;
    sphere.radius = startRadius;
    sphere.setColor(startColor);
  }

  void publish(ES::EventSystem* es, double time, double duration) override
  {
    double t = time / duration;
    double newRadius = (1.0 - t) * r0 + t * r1;
    auto newColor = Hologram::Colerp(c0, c1, t);

    sphere.radius = newRadius;
    sphere.setColor(newColor);

    sphere.publish(es);
  }

  void clear(ES::EventSystem* es) override
  {
    es->publish("HoloClear", sphere.name, sphere.topic);
    es->publish("HoloClear", sphere.name, sphere.topic);
  }

private:
  Sphere sphere;
  double r0, r1;
  std::vector<unsigned int> c0, c1;
};

/// Represents a collection of arrows pointing toward a common center point, aligned with some planar axis
/// Animates from one radius to another.
class ArrowRayTweener : public TweenPublisher
{
public:
  enum class Axis
  {
    XY,
    YZ,
    ZX
  };

  ArrowRayTweener(const std::string& name, const std::string& parent, double startRadius, double endRadius,
                  const std::vector<std::vector<unsigned int>> colorGradient, double colorCycleSpeed,
                  Axis axis=Axis::XY)
    : r0(startRadius), r1(endRadius), nameBase(name), parent(parent), colors(colorGradient),
      animationSpeed(colorCycleSpeed), axis(axis)
  {
    // generate points
    UpdateRays(startRadius, axis);
  }

  void publish(ES::EventSystem* es, double time, double duration) override
  {
    double t = time / duration;
    double newRadius = (1.0 - t) * r0 + t * r1;
    UpdateRays(newRadius, axis);

    for (auto& traj : trajectories)
    {
      traj.publish(es);
    }
  }

  void clear(ES::EventSystem* es) override
  {
    for (auto& traj : trajectories)
    {
      es->publish("HoloClear", traj.name, traj.topic);
      es->publish("HoloClear", traj.name, traj.topic);
    }
  }
private:

  void UpdateRays(double radius, Axis axis)
  {
    trajectories.clear();

    size_t nRays = 6;
    size_t nSteps = 6;
    for (size_t i = 0; i < nRays; i++)
    {
      double rx = cos(2.0*M_PI * (double)i/(double)nRays);
      double ry = sin(2.0*M_PI * (double)i/(double)nRays);

      auto traj = TrajectoryArrows(nameBase + "_" + std::to_string(i), parent);
      traj.setColors(colors);
      traj.animationSpeed = animationSpeed;

      for (size_t j = nSteps; j > 0; j--)
      {
        double rad = (double) j * radius / (double) nSteps;

        if (axis == Axis::XY)
          traj.points.push_back({rad*rx, rad*ry, 0});
        else if (axis == Axis::YZ)
          traj.points.push_back({0, rad*rx, rad*ry});
        else if (axis == Axis::ZX)
          traj.points.push_back({rad*ry, 0, rad*rx});
      }
      trajectories.push_back(traj);
    }
  }

  double r0, r1;
  std::string nameBase, parent;
  std::vector<std::vector<unsigned int>> colors;
  double animationSpeed;
  Axis axis;
  std::vector<TrajectoryArrows> trajectories;
};

/// Represents a collection of arrows pointing toward a common center point from the surface of a sphere
/// Animates from one radius to another.
class ArrowRaySphereTweener : public TweenPublisher
{
public:
  ArrowRaySphereTweener(const std::string& name, const std::string& parent, double startRadius, double endRadius,
                        const std::vector<std::vector<unsigned int>> colorGradient, double colorCycleSpeed,
                        std::string topic="ArrowRays")
    : r0(startRadius), r1(endRadius), namebase(name), parent(parent), colors(colorGradient),
      animationSpeed(colorCycleSpeed), holotopic(topic)
  {
    // generate points
    UpdateRays(startRadius);
  }

  void publish(ES::EventSystem* es, double time, double duration) override
  {
    double t = time / duration;
    double newRadius = (1.0 - t) * r0 + t * r1;
    UpdateRays(newRadius);


    for (auto& t : trajectories)
    {
      t.publish(es);
    }
  }

  void clear(ES::EventSystem* es) override
  {
    for (auto& t : trajectories)
    {
      es->publish("HoloClear", t.name, t.topic);
      es->publish("HoloClear", t.name, t.topic);
    }
    trajectories.clear();
  }
private:

  void UpdateRays(double radius)
  {
    for (auto& traj : trajectories)
    {
      traj.points.clear();
    }
    trajectories.clear();

    double phi = (1.0 + sqrt(5.0)) / 2.0;
    double a = 1.0 / sqrt(3.0);
    double b = a / phi;
    double c = a * phi;

    // Use points of a dodecahedron as a simplified set of spherical points
    // These are reasonably well-spaced
    std::vector<std::vector<double>> points;
    for (double i = -1; i <= 1; i+=2)
    {
      for (double j = -1; j<= 1; j+=2)
      {
        points.push_back({0,              i* c * radius, j* b * radius});
        points.push_back({i* c * radius, j* b * radius, 0});
        points.push_back({i* b * radius, 0,              j* c * radius});

        for (double k = -1; k <= 1; k+=2)
        {
          points.push_back({i* a * radius, j* a * radius, k* a * radius});
        }
      }
    }


    double stepSize = radius / 6.0;
    for (size_t i = 0; i < points.size(); i++)
    {
      auto t = TrajectoryArrows(namebase + "_" + std::to_string(i), parent);
      t.topic = holotopic;
      t.setColors(colors);
      t.animationSpeed = animationSpeed;

      Vec3d_normalizeSelf(points[i].data());
      for (size_t j = 6; j > 0; j--)
      {
        double r = (double)j * stepSize;
        t.points.push_back({r* points[i][0], r* points[i][1], r* points[i][2]});
      }

      trajectories.push_back(t);
    }

  }

  double r0, r1;
  std::string namebase, parent;
  std::vector<std::vector<unsigned int>> colors;
  double animationSpeed;
  std::vector<TrajectoryArrows> trajectories;
  std::string holotopic;
};



/// Represents a simple animated hologram
/// This will continuously update a hologram from one state to another, then self-destruct
/// After the animation begins playing, DO NOT TOUCH THE OBJECT
/// TODO: Come up with a better class name?
template<typename H>
class Tween : Rcs::PeriodicCallback
{
public:

  ///
  /// \param es               ES::EventSystem to publish updates through
  /// \param p                TweenPublisher to use
  /// \param duration         Duration of the effect
  /// \param updateFrequency  Frequency of updates to TweenPublisher
  static void StartNew(ES::EventSystem* es, H p, double duration, double updateFrequency=30)
  {
    auto t = new Tween<H>(es, p, duration);
    t->start(updateFrequency);
  }

private:
  Tween(ES::EventSystem* es, H p, double duration) : es(es), time(0), duration(duration), publisher(p)
  {
    RCHECK(duration >= 0);
  }

  Tween(const Tween&) = default;
  Tween& operator=(const Tween&) = default;

  void callback() override
  {
    time += 1.0 / getUpdateFrequency(); /// TODO: Support nonlinear time interpolation?

    publisher.publish(es, time, duration);

    if (time >= duration)
    {
      publisher.clear(es);
      stop();
      delete this;
    }
  }

  ES::EventSystem* es;
  double time, duration;
  H publisher;
};

#endif   // RCS_HOLOPARSER_H
