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

#ifndef RCS_POLYGONOBJECTPLANNER_H
#define RCS_POLYGONOBJECTPLANNER_H


#include "ComponentBase.h"
#include "PolygonObjectModel.h"

#include <ControllerBase.h>



namespace Rcs
{
/*! \brief Motion planner class for polygon objects.
 *
 *         The class subscribes to the following events:
 *         - PlanToAngleAndExecute: Plans from the current state to the
 *                                  published angle. If all succeeds, the
 *                                  trajectory constraints are generated and
 *                                  published through the SetTrajectory event.
 *                                  Planning is performed in a thread, so that
 *                                  the event loop will not be interrupted.
 *         - EmergencyStop: Sets an internal eStop flag. If this is set, no
 *                          trajectories will be published
 *         - EmergencyRecover: Sets the eStop flag to false.
 *         - InitFromState: Sets the state of the internal controller to the
 *                          one of the passed graph, and recomputes the
 *                          search state.
 *         - SetPolygon: Copies the published PolggonObjectModel into the
 *                       classes member.
 *         - MoveTo: Creates a trajectory to the given state. It is then
 *                   published through the SetTrajectory event, but only if
 *                   - eStop is not true
 *                   - no other trajectory is currently running
 *                   - the goal state is valid (see checkState() function)
 *                   - the passed goal state has 5 entries
 *         - MoveToInit: Computes leftmost and righmost support points, and
 *                       publishes this to the "MoveTo" event. The rotation
 *                       of the object is kept.
 *         - MoveToState: Publishes a trajectory that moves the system to
 *                        the desired 5d state.
 *         - TurnObject: Rotates the object to the desired value.
 *         - DriveHome: Releases all tasks except for the ones of the mobile
 *                      base, and returns the base to [0 0 0].
 *         - SetJointCommand: Updates the internal controller from the desired
 *                            graph's values.
 *         - TrajectoryMoving: Remembers if the trajectory is moving, or has
 *                             finished. New trajectories are only planned if
 *                             the trajectory has been finished.
 *
 *         The class publishes the following events:
 *         - SetTextLine: Publishes current state and its validity on text
 *                        line 0.
 *         - RemoveNode: Shows some OpenSceneGraph nodes to be shown in the
 *                       graphics window.
 *         - AddChildNode: Deletes some OpenSceneGraph nodes to be shown in
 *                         the graphics window.
 *         - SetTrajectory: Upon successful planning, the generated
 *                          trajectory constraint is published.
 *         - PlanToAngleAndExecute: In eternal test only
 *
 */
class PolygonObjectPlanner : public ComponentBase
{
public:

  /*! \brief Constructs an instance of the planner. The controller is used to
   *         compute the search state according to the task space coordinates.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables. It will be
   *                       cloned.
   * \param[in] deltaPhi   Angular discretization of the search state.
   */
  PolygonObjectPlanner(EntityBase* parent,
                       const ControllerBase* controller,
                       double deltaPhi);

  /*! \brief Computes the search state from the state corresponding to the
   *         commands ("desired state"). The name might be a bit misleading.
   *
   *  \return 5d vector: Rotation of polygon, right robot hand, left robot hand,
   *          right partner hand, left partner hand.
   */
  std::vector<int> getCurrentState() const;

  /*! \brief Returns the angular discretization of the state that corresponds
   *         to the rotation of the polygon.
   */
  double getDeltaPhi() const;

  /*! \brief Publishes an event that leads to an endless test rotating a
   *         polygon back and forth.
   */
  void startEndlessTest();

private:

  void subscribeAll();
  void onPlanner(double phi, double ttc);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onPlanFromTo(std::string from, std::string to);
  void onPlanToStateAndExecute(std::string to, double ttc);
  void onSetPolygon(PolygonObjectModel model);
  void onMoveToState(std::string to, double ttc);
  void onJointCommand(const MatNd* q_des);
  void onMoveToState(std::vector<int> to, double ttc);
  void onMoveToStateStr(std::string to, double ttc);
  void onMoveToInitialState(double ttc);
  void onTurnObject(int phi);
  void onDriveHome(double ttc);
  void onEternalTest(bool started);
  void onUpdateMovingState(bool moves);
  void showContacts(const std::vector<HTr>& contacts,
                    const std::string& nodeName,
                    const std::string& parent);

  void plannerThread(std::vector<int> from,
                     std::vector<int> to,
                     double ttc);

  bool checkState(std::vector<int> state) const;

  PolygonObjectModel objectModel;
  ControllerBase controller;
  double deltaPhi;
  bool eStop;
  bool isMoving;
  std::vector<int> searchState;

  PolygonObjectPlanner(const PolygonObjectPlanner&);
  PolygonObjectPlanner& operator=(const PolygonObjectPlanner&);
};

}

#endif   // RCS_POLYGONOBJECTPLANNER_H
