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


#ifndef RCS_MONITORREQUEST_H
#define RCS_MONITORREQUEST_H

#include <string>
#include <map>
#include <vector>
#include <memory>


namespace Rcs
{

class MonitorRequest;
typedef std::shared_ptr<MonitorRequest> MonitorRequest_ptr;

/*!
 *  \brief MonitorSignal contains a sensor signal with a threshold and a condition (less than or larger/equal)
 */
class MonitorSignal
{
public:
  /*!
   *  \brief MonitorSignal contains a sensor signal with a threshold and a condition (less than or larger/equal)
   *
   *  \param[in] signal_in                  Signal name/identifier (string)
   *  \param[in] threshold_in               Threshold value
   *  \param[in] lesserThan_in              Boolean (true = less than threshold, false = larger or equal)
   *  \param[in] progressThreshold_in       Defines bracket around threshold in which progress is shown towards fulfillment
   */
  MonitorSignal(std::string signal_in, double threshold_in, bool lesserThan_in = true, double progressThreshold_in = 0.2);
  ~MonitorSignal();

  std::string signal;
  double threshold;
  bool lesserThan;
  double progressThreshold;
};

/*!
 *  \brief A MonitorRequest contains several sensor signals with a threshold. Once all signal conditions are fulfilled
 *         for the required activation duration an event can be emitted on the defined topic.
 */
class MonitorRequest
{
public:
  /*!
   *  \brief A MonitorRequest contains several sensor signals with a threshold. Once all signal conditions are fulfilled
   *         for the required activation duration an event can be emitted on the defined topic.
   */
  MonitorRequest();

  /*!
   *  \brief A MonitorRequest contains several sensor signals with a threshold. Once all signal conditions are fulfilled
   *         for the required activation duration an event can be emitted on the defined topic.
   *
   *  \param[in] topic_in                   Topic for event to be emitted
   *  \param[in] id_in                      ID for the request
   *  \param[in] sequence_num               Sequence number to prevent race conditions and avoid expired signals
   *  \param[in] desc                       Text description for the request
   *  \param[in] signal_duration            Defines how long all conditions have to be fulfilled before success
   */
  MonitorRequest(std::string topic_in, int id_in, int sequence_num, std::string desc = "", double signal_duration = 0.0);

  /*!
   *  \brief A MonitorRequest contains several sensor signals with a threshold. Once all signal conditions are fulfilled
   *         for the required activation duration an event can be emitted on the defined topic.
   *
   *  \param[in] in                   Shared pointer to other MonitorRequest to copy from
   */
  MonitorRequest(std::shared_ptr<MonitorRequest> in);

  /*! \brief Nothing to be done in the destructor.
   */
  ~MonitorRequest();

  /*!
   *  \brief Adds a MonitorSignal to the MonitorRequest.
   *
   *  \param[in] signal_in                  Name/identifier (string)
   *  \param[in] threshold_in               Threshold value
   *  \param[in] lesserThan_in              Boolean (true = less than threshold, false = larger or equal)
   *  \param[in] progressThreshold_in       Defines bracket around threshold in which progress is shown towards fulfillment
   */
  void addSignal(std::string signal_in, double threshold_in, bool lesserThan_in = true, double progressThreshold_in = 0.2);

  /*!
   *  \brief Compares the MonitorRequest with another one.
   *
   *  \param[in] other           MonitorRequest to compare to.
   *  \return                    Boolean if they have the same topic, id, and sequence number
   */
  bool operator==(const MonitorRequest& other);

  /*!
   *  \brief Creates a short string represenation of the request.
   *
   *  \return                    String representation
   */
  std::string toString();

  /*!
   *  \brief Creates a long string represenation of the request with all MonitorSignals.
   *
   *  \return                    String representation
   */
  std::string toStringLong();

  /*!
   *  \brief Evaluate the specified MonitorSignal given the provided sensor data
   *
   *  \param[in] idx             Index of the MonitorSignal to be evaluated
   *  \param[in] sensorData      std::map of the used sensor signals with string id and value
   *  \return                    -1.0 if invalid, otherwise between 1.0 (fulfilled) and 0.0 (unfulfilled) based on
   *                             progressThreshold value
   */
  double evaluate(int idx, std::map<std::string, double>& sensorData);

  /*!
   *  \brief Evaluate all MonitorSignals given the provided sensor data
   *
   *  \param[in] sensorData      std::map of the used sensor signals with string id and value
   *  \param[in] time            Current time
   *  \return                    -1.0 if invalid, otherwise between 1.0 (fulfilled) and 0.0 (unfulfilled) based on
   *                             progressThreshold value
   */
  double evaluate2(std::map<std::string, double>& sensorData, double time);

  /*!
   *  \brief Prints a complete status of the MonitorRequest for the current sensor data and time.
   *
   *  \param[in] sensorData      std::map of the used sensor signals with string id and value
   *  \param[in] time            Current time
   */
  void printStatus(std::map<std::string, double>& sensorData, double time);

  //TODO: these variables should be private with setters and getters
  /*!
   *  \brief Topic for event to be emitted on success
   */
  std::string topic;

  /*!
   *  \brief Sequence number to prevent race conditions and avoid expired signals
   */
  int sequenceNum;

  /*!
   *  \brief ID for the request
   */
  int id;

  /*!
   *  \brief Defines how long all conditions have to be fulfilled before success
   */
  double activationDurationThreshold;

  /*!
   *  \brief Duration this request has been fulfilled already
   */
  double activationDuration;

  /*!
   *  \brief Text description for the request
   */
  std::string description;

  /*!
   *  \brief Time when request was created
   */
  double creationTime;

  /*!
   *  \brief Last time this request was NOT fulfilled
   */
  double lastDeactivationTime;

  /*!
   *  \brief Earliest time this request can be fulfilled
   */
  double minTime;

  /*!
   *  \brief Latest time this request can be fulfilled
   */
  double maxTime;

  /*!
   *  \brief Vector of all MonitorSignals
   */
  std::vector<MonitorSignal> sensorSignals;
};

}

#endif //RCS_MONITORREQUEST_H
