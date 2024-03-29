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

#ifndef DC_ENTITYBASE_H
#define DC_ENTITYBASE_H

#include <EventSystem.h>
#include <Rcs_graph.h>

#include <pthread.h>


namespace Dc
{
/*! \brief Class wrapping event-loop specific functionality, Maintains some
 *         members related to timings, such as the time step dt, a flag
 *         indicating if the loop is paused, some statistice and a mutex
 *         that ensures no issues when accessed concurrently.
 *
 *         This class deserves some severe rethinking. When designing an
 *         Entity Component System, several entities should coexist. Therefore,
 *         this class, even though convenient, should better be renamed or
 *         restructured.
 *
 *         The class subscribes to the following events:
 *         - TogglePause: Toggles the pause member variable. If it is set to
 *                        true, the process() call will wait for the enter
 *                        key before continuing.
 *         - ToggleTimeFreeze: Toggles the timeFreeze member variable. This has
 *                             no effect inside this class (\todo: should be
 *                             fixed)
 *         - Print: Prints some class information to the console.
 */
class EntityBase : public ES::EventSystem
{
public:

  /*! \brief Sets all members to defaults. The default time step is 0.05.
   */
  EntityBase();

  /*! \brief Releases the mutex. \todo: Should this be a std::mutex?
   */
  virtual ~EntityBase();

  /*! \brief Returns the time step between two calls to process()
   */
  double getDt() const;

  /*! \brief Sets the time step between two calls to process(). No checking is
   *         done.
   */
  void setDt(double dt_);

  /*! \brief Returns the accumulated time. It is incremented by dt in each call
   *         to stepTime()
   */
  double getTime() const;

  /*! \brief Sets the classes internal time to the given value.
   */
  void setTime(double time_);

  /*! \brief Increments the internal time by the internal dt.
   */
  void stepTime();

  /*! \brief Processes the event look and waits for enter key if pause mode
   *         is activated.
   */
  void process();

  /*! \brief Returns the size of the event queue at the time of the call. If
   *         it is queries right after the process() call, it will be empty.
   */
  size_t queueSize() const;

  /*! \brief Returns the all-time maximum queue size since construction of this
   *         class.
   */
  size_t getMaxQueueSize() const;

  /*! \brief Returns true if the timeFrozen member is true, false otherwise.
   *         The timeFrozen member has no effect inside this class.
   */
  bool getTimeFrozen() const;

  /*! \brief Returns true if the emergency stop callback has been called, false
   *         otherwise.
   */
  bool getEmergencyStopFlag() const;

  bool initialize(RcsGraph* graph);

  std::string printToString() const;

private:

  void onTogglePause();
  void onToggleTimeFrozen();
  void onPrint();
  void onEmergencyStop();
  void onEmergencyRecover();

  double dt;
  double time;
  bool pause;
  bool timeFrozen;
  bool eStop;
  size_t maxQueueSize;
  mutable pthread_mutex_t mutex;
};

}

#endif   // DC_ENTITYBASE_H
