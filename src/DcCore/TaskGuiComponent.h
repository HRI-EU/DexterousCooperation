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

#ifndef DC_TASKGUICOMPONENT_H
#define DC_TASKGUICOMPONENT_H


#include "ComponentBase.h"

#include <ControllerBase.h>
#include <Rcs_filters.h>


namespace Dc
{

/*! \brief Task gui class. Runs a TaskWidget in its own thread, reads in
 *         the slider values, filters them, and provides them as command
 *         vectors in a MatNd. Further, the gui displays the current task
 *         values communicated through the ComputeKinematics event.
 *
 *         The class publishes no events.
 *
 *         The class subscribes to the following events:
 *         - Start: Launches a TaskWidget. This subscriber does not check if
 *                  another widget is already running.
 *         - Stop: Destroys all running TaskWidgets
 *         - InitFromState: Sets the slider values of all running guis
 *                          to the values of the passed graph.
 *         - ComputeKinematics: Iterates all desired value filters and updates
 *                              the corresponding values in the gui.
 *         - SetTaskCommand: Copies the incoming activation and desired task
 *                           values to the gui. This is only done if the gui
 *                           has been set to passive.
 * \todo: Avoid subscribing to SetTaskCommand several times. Also, a_des is currently not protected against concurrent Gui access. We should have a copy of it that is updated in the setState method.
 */
class TaskGuiComponent : public ComponentBase
{
  friend class TaskUpdateCallback;

public:

  /*! \brief Constructs a TaskGuiComponent instance and subscribes to all
   *         events described above. The constructor does not launch a gui.
   *         This happens only through the Start event.
   *
   *  \param[in] parent     Entity class responsible for event subscriptions
   *  \param[in] controller Controller instance, copied by this class. If the
   *                        controller has been instantiated from a xml file,
   *                        this classes activation vector is set according to
   *                        the entries in the xml file (active-tag)
   */
  TaskGuiComponent(EntityBase* parent, const Rcs::ControllerBase* controller);

  /*! \brief Unsubscribes all subscribers and frees all memory. Also cleans
   *         up mutex.
   */
  virtual ~TaskGuiComponent();

  /*! \brief Returns a pointer to the internal activation array. This can be
   *         passed to other class instances to pass on the gui values. The
   *         values are updated in the ComputeKinematics event. Access should
   *         be done only through the event loop to avoid concurrent access.
   */
  const MatNd* getActivationPtr() const;

  /*! \brief Returns a pointer to the internal desired task vector. This can be
   *         passed to other class instances to pass on the gui values. The
   *         values are updated in the ComputeKinematics event. Access should
   *         be done only through the event loop to avoid concurrent access.
   *         The task vector undergoes a ramp filtering with values that can
   *         be looked up in the constructor implementation.
   */
  const MatNd* getTaskCommandPtr() const;

  /*! \brief Needs to be called before the start event. Then, the class
   *         subscribes to the SetTaskCommand event and copies the passed
   *         arrays to the gui. This makes it possible to inspect the
   *         currently set task and activation commands in the gui.
   */
  void setPassive(bool enable);

private:

  void onTaskCommand(const MatNd* a, const MatNd* x);
  void guiCallback();
  void setState(RcsGraph* from);
  void onInitFromState(const RcsGraph* target);
  void start();
  void stop();

  Rcs::ControllerBase controller;
  MatNd* a_des;
  MatNd* x_des;
  MatNd* x_des_filt;
  MatNd* x_curr;
  Rcs::RampFilterND* filt;
  std::vector<int> handle;
  mutable pthread_mutex_t mtx;
  bool passive;

  TaskGuiComponent(const TaskGuiComponent&);
  TaskGuiComponent& operator=(const TaskGuiComponent&);
};

}

#endif   // DC_TASKGUICOMPONENT_H
