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

#ifndef RCS_INTENTIONVISUALIZATIONCOMPONENT_H
#define RCS_INTENTIONVISUALIZATIONCOMPONENT_H

#include "WheelObjectModel.h"
#include "BoxObjectModel.h"
#include "ComponentBase.h"
#include <string>
#include <algorithm>

namespace Rcs
{

template <typename T>
class IntentionVisualizationComponent : public ComponentBase
{
public:
  IntentionVisualizationComponent(EntityBase* es) : Rcs::ComponentBase(es)
  {
    onInit();
    getEntity()->subscribe("ClearMonitors", [this](std::string type)
    {
      if (type == "Intention" || type == "all")
      {
        progressState.clear();
      }
    });

    getEntity()->subscribe("MonitorStatusChanged", [this](std::string topic, int id, int seqNum, double progress)
    {
      if (topic == "Intention")
      {
        this->onIntentionMonitorStatusChanged((T) id, seqNum, progress);
      }
      //else
      {
        this->onMonitorStatusChanged(topic, id, seqNum, progress);
      }
    });

    // visualize intention confirmation
    getEntity()->subscribe("Intention", [this](int intention, int something)
    {
      this->onIntention((T) intention);
    });

    // update visualizations if object model changes
    getEntity()->subscribe("ObjectModelChanged",
                           [this](std::vector<HTr> roboGraspPoints, std::vector<HTr> partnerGraspPoints)
    {
      this->onObjectModelChanged(roboGraspPoints, partnerGraspPoints);
    });

    getEntity()->subscribe("AAA Re-init Visualization", [this]()
    {
      onInit();
    });
  }

  IntentionVisualizationComponent(EntityBase* es, double threshold) : IntentionVisualizationComponent(es)
  {
    publishThreshold = threshold;
  }

private:
  struct IntentionProgress
  {
    T intention;
    double currProgress;
    double prevProgress;
  };

  void onInit() {}
  void onIntention(T intention) {}
  void onIntentionMonitorStatusChanged(T intention, int seqNum, double progress)
  {
    if (progressState.find(seqNum) != progressState.end())
    {
      progressState[seqNum].currProgress = progress;

      if (fabs(progressState[seqNum].currProgress - progressState[seqNum].prevProgress) > publishThreshold)
      {
        //RLOG(0, "INTENTION STATUS CHANGED: %d (%d): %5.2f  (diff: %5.2f)", intention, seqNum, progress, fabs(progressState[seqNum].currProgress - progressState[seqNum].prevProgress));
        progressState[seqNum].prevProgress = progressState[seqNum].currProgress;

        // if a monitor has changed, find the new max value
        auto highest_progress = std::max_element(progressState.begin(), progressState.end(), [](const typename decltype(progressState)::value_type& a, const typename decltype(progressState)::value_type& b)
        {
          return a.second.currProgress < b.second.currProgress;
        });

        //RLOG(0, "%d intention %d has highest progress: %5.2f",
        // highest_progress->first, highest_progress->second.intention, highest_progress->second.currProgress);
        showIntentionProgress(highest_progress->second.intention, highest_progress->second.currProgress);
      }
    }
    else
    {
      progressState[seqNum] = {intention, progress, progress};
    }
  }

  void showIntentionProgress(T intention, double progress) {}
  void onMonitorStatusChanged(const std::string& topic, int id, int seqNum, double progress) {}
  void onObjectModelChanged(std::vector<HTr>& roboGraspPoints, std::vector<HTr>& partnerGraspPoints) {}

  double publishThreshold = 0.01;
  std::map<int, IntentionProgress> progressState;
};

/*************************************
 *    WheelObject Intentions
 ************************************/

template<>
void IntentionVisualizationComponent<WheelObjectModel::Intention>::onInit()
{
  // ensure the Wheel transform is tracked before we try to attach any holograms to it
  getEntity()->publish("HoloTrackTransform", std::string("Wheel"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("Vicon tire"), GraphType::Current);

  Cylinder c("WheelGeom", "Vicon tire", {232, 32, 100, 120});
  c.topic = "WheelOcclusion";
  c.radius = 0.36*0.8;
  c.length = 0.12*0.8;
  getEntity()->publish("HoloUpdateCylinder", c);

  CylinderOccluder co("WheelOccluder", "Wheel", {0, 0, 0});
  c.topic = "WheelOcclusion";
  co.radius = 0.36;
  co.length = 0.12;
  getEntity()->publish("HoloUpdateCylinderOccluder", co);
}

template<>
void IntentionVisualizationComponent<WheelObjectModel::Intention>::onIntention(WheelObjectModel::Intention wheelIntent)
{
  Sprite sprite("intention_sprite", HoloComponent::DIRECT_TRANSFORM(), {50, 200, 70}, 2.0);
  sprite.topic = "Intentions";
  sprite.scale = 0.1;
  sprite.targetTransform = "Wheel";

  switch (wheelIntent)
  {
    case WheelObjectModel::Intention::ROTATE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case WheelObjectModel::Intention::ROTATE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case WheelObjectModel::Intention::FLIP_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case WheelObjectModel::Intention::FLIP_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case WheelObjectModel::Intention::MOVE_UP:
      sprite.spriteName = "icons/arrows/ArrowUp_outline";
      break;
    case WheelObjectModel::Intention::MOVE_DOWN:
      sprite.spriteName = "icons/arrows/ArrowDown_outline";
      break;
    case WheelObjectModel::Intention::MOVE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowLeft_outline";
      break;
    case WheelObjectModel::Intention::MOVE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRight_outline";
      break;
    default:
      RWARNING(0, "Unexpected WheelObjectModel::Intention value: %d", (int)wheelIntent);
      break;
  }
  getEntity()->publish("HoloUpdateSprite", sprite);
}

template<>
void IntentionVisualizationComponent<WheelObjectModel::Intention>::
showIntentionProgress(WheelObjectModel::Intention intention, double progress)
{
  Sprite sprite("intention_sprite", HoloComponent::DIRECT_TRANSFORM(), {200, 200, 70,
                                                                        (unsigned int)(progress*255.0)
                                                                       }, 2.0);
  sprite.topic = "Intentions";
  sprite.scale = 0.07;
  sprite.targetTransform = "Wheel";
  switch (intention)
  {
    case WheelObjectModel::Intention::ROTATE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case WheelObjectModel::Intention::ROTATE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case WheelObjectModel::Intention::FLIP_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case WheelObjectModel::Intention::FLIP_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case WheelObjectModel::Intention::MOVE_UP:
      sprite.spriteName = "icons/arrows/ArrowUp_outline";
      break;
    case WheelObjectModel::Intention::MOVE_DOWN:
      sprite.spriteName = "icons/arrows/ArrowDown_outline";
      break;
    case WheelObjectModel::Intention::MOVE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowLeft_outline";
      break;
    case WheelObjectModel::Intention::MOVE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRight_outline";
      break;
    default:
      RWARNING(0, "Unexpected WheelObjectModel::Intention value: %d", (int)intention);
      break;
  }
  getEntity()->publish("HoloUpdateSprite", sprite);

  HTr p_off;
  p_off.org[0] = -0.001; // small hack to prevent z-fighting if the progress sprite overlaps the intention sprite
  getEntity()->publish("HoloUpdateTransform", p_off, std::string("intention_progress_offset"),
                       HoloComponent::DIRECT_TRANSFORM() /*std::string("Box_real_center")*/);
  ProgressSprite psprite("intention_progress", "intention_progress_offset", 2.0);
  psprite.topic = "Intentions";
  psprite.progress = progress;
  psprite.setColor(Hologram::Colerp({250, 250, 250, 94}, {50, 250, 50, 128}, progress));
  psprite.spriteName = "progress/circle03";
  getEntity()->publish("HoloUpdateProgressSprite", psprite);
}

/*************************************
 *    BoxObject Intentions
 ************************************/

template<>
void IntentionVisualizationComponent<BoxObjectModel::Intention>::onInit()
{
  // ensure the Box transform is tracked before we try to attach any holograms to it
  getEntity()->publish("HoloTrackTransform", std::string("Box_real_center"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("Box_v"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("Vicon box"), GraphType::Current);
  getEntity()->publish("HoloTrackTransform", std::string("Partner_Box_v"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("PartnerGrasp_R"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("PartnerGrasp_L"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("PowerGrasp_R"), GraphType::Desired);
  getEntity()->publish("HoloTrackTransform", std::string("PowerGrasp_L"), GraphType::Desired);

  getEntity()->publish("HoloTrackTransform", std::string("Vicon hand right"), GraphType::Current);
  getEntity()->publish("HoloTrackTransform", std::string("Vicon hand left"), GraphType::Current);
  getEntity()->publish("HoloTrackTransform", std::string("Hololens_1"), GraphType::Current);
  getEntity()->publish("HoloTrackTransform", std::string("Hololens_SlamFrame"), GraphType::Current);

  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Partner_Box_v(mirror)"),
                       std::string("Partner_Box_v"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("PartnerGrasp_R(mirror)"),
                       std::string("PartnerGrasp_R"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("PartnerGrasp_L(mirror)"),
                       std::string("PartnerGrasp_L"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Box_v(mirror)"), std::string("Box_v"),
                       std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Box_real_center(mirror)"),
                       std::string("Box_real_center"), std::vector<double>({1, 0, 0}));

  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Vicon hand right(mirror)"),
                       std::string("Vicon hand right"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Vicon hand left(mirror)"),
                       std::string("Vicon hand left"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Hololens_SlamFrame(mirror)"),
                       std::string("Hololens_SlamFrame"), std::vector<double>({1, 0, 0}));
  getEntity()->publish("HoloUpdateMirrorTransform", std::string("Hololens_1(mirror)"),
                       std::string("Hololens_1"), std::vector<double>({1, 0, 0}));

  StaticModel rs = StaticModel("PartnerTrackedHandR", "Vicon hand right(mirror)", {4, 64, 78});
  rs.model = "Human_RightHand";
  rs.topic = "HumanSuggestion";
  getEntity()->publish("HoloUpdateStaticModel", rs);
  StaticModel ls = StaticModel("PartnerTrackedHandL", "Vicon hand left(mirror)", {116, 3, 29});
  ls.model = "Human_LeftHand";
  ls.topic = "HumanSuggestion";
  getEntity()->publish("HoloUpdateStaticModel", ls);

  StaticModel hl = StaticModel("HololensMirrorModel", "Hololens_SlamFrame(mirror)", {255, 255, 255});
  hl.model = "Hololens";
  hl.topic = "HumanSuggestion";

  getEntity()->publish("HoloUpdateStaticModel", hl);
}

template<>
void IntentionVisualizationComponent<BoxObjectModel::Intention>::
showIntentionProgress(BoxObjectModel::Intention boxIntent, double progress)
{
  Sprite sprite("intention_sprite", HoloComponent::DIRECT_TRANSFORM(),
  {200, 200, 70, (unsigned int)(progress*255.0)}, 2.0);
  sprite.topic = "Intentions";
  sprite.scale = 0.15;
  sprite.targetTransform = "Box_real_center";

  switch (boxIntent)
  {
    case BoxObjectModel::Intention::ROTATE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case BoxObjectModel::Intention::ROTATE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case BoxObjectModel::Intention::TILT_LEFT:
      break;
    case BoxObjectModel::Intention::TILT_RIGHT:
      break;
    default:
      RWARNING(0, "Unexpected WheelObjectModel::Intention value: %d", (int)boxIntent);
      break;
  }
  getEntity()->publish("HoloUpdateSprite", sprite);

  HTr* p_off = HTr_clone(HTr_identity());
  p_off->org[0] = -0.001; // small hack to prevent z-fighting if the progress sprite overlaps the intention sprite
  getEntity()->publish("HoloUpdateTransform", *p_off, std::string("intention_progress_offset"),
                       HoloComponent::DIRECT_TRANSFORM());
  RFREE(p_off);

  ProgressSprite psprite("intention_progress", "intention_progress_offset", 2.0);
  psprite.topic = "Intentions";
  psprite.progress = (progress-0.5)/0.5;
  psprite.setColor(Hologram::Colerp({250, 250, 250, 94}, {50, 250, 50, 128}, (progress-0.5)/0.5));
  psprite.spriteName = "progress/circle03";
  getEntity()->publish("HoloUpdateProgressSprite", psprite);
}

template<>
void IntentionVisualizationComponent<BoxObjectModel::Intention>::onIntention(BoxObjectModel::Intention boxIntent)
{
  Sprite sprite("intention_sprite", HoloComponent::DIRECT_TRANSFORM(), {50, 200, 70}, 2.0);
  sprite.scale = 0.18;
  sprite.targetTransform = "Box_real_center";

  switch (boxIntent)
  {
    case BoxObjectModel::Intention::ROTATE_RIGHT:
      sprite.spriteName = "icons/arrows/ArrowRotateLeft_outline";
      break;
    case BoxObjectModel::Intention::ROTATE_LEFT:
      sprite.spriteName = "icons/arrows/ArrowRotateRight_outline";
      break;
    case BoxObjectModel::Intention::TILT_LEFT:
      break;
    case BoxObjectModel::Intention::TILT_RIGHT:
      break;
    default:
      RWARNING(0, "Unexpected WheelObjectModel::Intention value: %d", (int)boxIntent);
      break;
  }
  getEntity()->publish("HoloUpdateSprite", sprite);
}

template<>
void IntentionVisualizationComponent<BoxObjectModel::Intention>::
onObjectModelChanged(std::vector<HTr>& roboGraspPoints,std::vector<HTr>& partnerGraspPoints)
{
  Poly2D obj("ObjectGraspSurface", "Box_v");
  obj.topic = "RobotInfo";
  obj.setPoints(roboGraspPoints);
  obj.setNormal(0, 0, 1);
  obj.setColor(135, 200, 152, 64);
  getEntity()->publish("HoloUpdatePoly2D", obj);

  // robot grasp points
  for (size_t i = 0; i < roboGraspPoints.size(); i++)
  {
    getEntity()->publish("HoloUpdateTransform",
                         roboGraspPoints[i], "RoboBoxGrasp_" + std::to_string(i), std::string("Box_v"));
    Sprite graspSprite("RoboBoxGrasp_" + std::to_string(i) + "_sprite", "RoboBoxGrasp_" + std::to_string(i));
    graspSprite.topic = "RobotInfo";
    graspSprite.scale = 0.05;
    graspSprite.spriteName =  "icons/crosshairs/crosshair117";
    graspSprite.setColor({128, 128, 128, 128});
    getEntity()->publish("HoloUpdateSprite", graspSprite);
  }

  // create center box transform from partner's mirrored perspective
  HTr partner_box_center;
  partner_box_center.org[2] = -0.5;
  getEntity()->publish("HoloUpdateTransform", partner_box_center, std::string("Box_partner_center(mirror)"),
                       std::string("Partner_Box_v"));

  for (size_t i = 0; i < partnerGraspPoints.size(); i++)
  {
    getEntity()->publish("HoloUpdateTransform", partnerGraspPoints[i], "PartnerBoxGrasp_" + std::to_string(i),
                         std::string("Partner_Box_v"));
    getEntity()->publish("HoloUpdateTransform", partnerGraspPoints[i], "PartnerBoxGrasp(mirror)_" + std::to_string(i),
                         std::string("Partner_Box_v(mirror)"));
    Sprite graspSprite("PartnerBoxGrasp_" + std::to_string(i) + "_sprite", "PartnerBoxGrasp_" + std::to_string(i));
    graspSprite.topic = "HumanSuggestion";
    graspSprite.scale = 0.05;
    graspSprite.spriteName =  "icons/crosshairs/crosshair031"; // + std::to_string(46+i);
    graspSprite.setColor({128, 128, 128, 128});
    getEntity()->publish("HoloUpdateSprite", graspSprite);

    Sprite graspSpritem("PartnerBoxGrasp(mirror)_" + std::to_string(i) + "_sprite",
                        "PartnerBoxGrasp(mirror)_" + std::to_string(i));
    graspSpritem.topic = "HumanSuggestion";
    graspSpritem.scale = 0.05;
    graspSpritem.spriteName =  "icons/crosshairs/crosshair031"; // + std::to_string(46+i);
    graspSpritem.setColor({128, 128, 128, 128});
    graspSprite.publish(getEntity());
  }
}

} // namespace Rcs

#endif // RCS_INTENTIONVISUALIZATIONCOMPONENT_H
