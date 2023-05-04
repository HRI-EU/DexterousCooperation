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

#include "MonitorRequest.h"
#include "Rcs_macros.h"

#include <algorithm>



namespace Dc
{

MonitorSignal::MonitorSignal(std::string signal_in, double threshold_in, bool lesserThan_in,
                             double progressThreshold_in)
{
  signal = signal_in;
  threshold = threshold_in;
  lesserThan = lesserThan_in;
  progressThreshold = progressThreshold_in;

  //    RLOG(0, "New MonitorSignal: %s, progress: %5.3f", signal.c_str(), progressThreshold);
}

MonitorSignal::~MonitorSignal() {}



MonitorRequest::MonitorRequest()
{
  topic = "";
  sequenceNum = 0;
  activationDurationThreshold = 0.0;
  activationDuration = 0.0;
  description = "";
  sensorSignals.clear();
  minTime = 0.0;
  maxTime = 0.0;
  creationTime = 0.0;
  lastDeactivationTime = 0.0;
}

MonitorRequest::MonitorRequest(std::string topic_in, int id_in, int sequence_num, std::string desc,
                               double signal_duration)
{
  topic = topic_in;
  id = id_in;
  sequenceNum = sequence_num;
  activationDurationThreshold = signal_duration;
  activationDuration = 0.0;
  description = desc;
  sensorSignals.clear();
  minTime = 0.0;
  maxTime = 0.0;
  creationTime = 0.0;
  lastDeactivationTime = 0.0;
}

MonitorRequest::MonitorRequest(std::shared_ptr<MonitorRequest> in)
{
  id = in->id;
  topic = in->topic;
  sequenceNum = in->sequenceNum;
  activationDurationThreshold = in->activationDurationThreshold;
  activationDuration = in->activationDuration;
  description = in->description;

  //    sensorSignals.clear();
  sensorSignals = in->sensorSignals;
  minTime = in->minTime;
  maxTime = in->maxTime;
  creationTime = in->creationTime;
  lastDeactivationTime = in->lastDeactivationTime;
}


MonitorRequest::~MonitorRequest()
{}

void MonitorRequest::addSignal(std::string signal_in, double threshold_in, bool lesserThan_in,
                               double progressThreshold_in)
{
  MonitorSignal tmp(signal_in, threshold_in, lesserThan_in, progressThreshold_in);
  sensorSignals.push_back(tmp);
}

bool MonitorRequest::operator==(const MonitorRequest& other)
{
  if (topic != other.topic)
  {
    return false;
  }
  if (sequenceNum != other.sequenceNum)
  {
    return false;
  }
  if (id != other.id)
  {
    return false;
  }

  return true;
}

std::string MonitorRequest::toString()
{
  std::string txt = topic + " " + std::to_string(id) + " " + std::to_string(sequenceNum);
  return txt;
}

std::string MonitorRequest::toStringLong()
{
  std::string txt = toString();
  txt += "\n";

  for (int i = 0; i < (int) sensorSignals.size(); i++)
  {
    txt += "\t";
    txt += std::to_string(i+1) + ": ";
    txt += sensorSignals[i].signal + (sensorSignals[i].lesserThan ? " < " : " >= ") +
           std::to_string(sensorSignals[i].threshold);
    txt += "\n";
  }
  return txt;
}

double MonitorRequest::evaluate(int idx, std::map<std::string, double>& sensorData)
{
  if (idx < 0 || idx >= (int) sensorSignals.size())
  {
    return 0.0;
  }



  auto it = sensorData.find(sensorSignals[idx].signal);
  if (it==sensorData.end())
  {
    //      printf("Sensor signal '%s' not found.\n", sensorSignals[idx].signal.c_str());
    return -1.0;
  }

  double result = fabs(it->second - sensorSignals[idx].threshold);

  //    if (it->first == "hand_pos_R_x")
  //    {
  //        RLOG(0, " Signal: %s, %5.3f, %5.3f, threshold: %5.3f, dist: %5.3f", it->first.c_str(),
  //it->second, sensorSignals[idx].threshold, sensorSignals[idx].progressThreshold, result);
  //    }

  if (result > sensorSignals[idx].progressThreshold)
  {
    result = 0.0;
  }
  else
  {
    result = 1.0 - (result / sensorSignals[idx].progressThreshold);
  }

  //    printf("  %d: signal: %s, value: %5.2f  %s threshold: %5.2f, \n",
  //        idx, sensorSignals[idx].signal.c_str(), it->second, sensorSignals[idx].lesserThan? "<":">",
  //sensorSignals[idx].threshold  );

  if (sensorSignals[idx].lesserThan)
  {
    if (it->second < sensorSignals[idx].threshold)
    {
      result = 1.0;
    }
  }
  else
  {
    if (it->second >= sensorSignals[idx].threshold)
    {
      result = 1.0;
    }
  }
  return result;
}

//  int evaluate(std::map<std::string, double>& sensorData, double dt = 0.0)
//  {
//    bool conditionsMet = true;
//
//    for (size_t s = 0; s < sensorSignals.size(); s++)
//    {
//      int satisfied = evaluate((int) s, sensorData);
//      if (satisfied == -1)
//      {
//        return -1;
//      }
//      if (satisfied == 0)
//      {
//        conditionsMet = false;
//        break;
//      }
//    }
//
//    if (conditionsMet)
//    {
//      activationDuration += dt;
//
//      if (activationDuration >= activationDurationThreshold)
//      {
//        return true;
//      }
//    }
//    else
//    {
//      activationDuration = 0.0;
//    }
//
//    return false;
//  }

double MonitorRequest::evaluate2(std::map<std::string, double>& sensorData, double time)
{
  bool conditionsMet = true;
  double activeTotal = 0.0;

  for (size_t s = 0; s < sensorSignals.size(); s++)
  {
    double satisfied = evaluate((int) s, sensorData);
    activeTotal += satisfied;

    if (satisfied < 0.0)
    {
      return -1.0;
    }
    if (satisfied < 1.0)
    {
      conditionsMet = false;
      //        break;
    }
  }

  if (conditionsMet)
  {
    activationDuration = time - lastDeactivationTime;

    if (activationDuration >= activationDurationThreshold)
    {
      return 1.0;
    }
  }
  else
  {
    lastDeactivationTime = time;
    activationDuration = 0.0;
  }

  double percentComplete = activeTotal / (double) sensorSignals.size();

  percentComplete = std::max(percentComplete, 0.0);
  percentComplete = std::min(percentComplete, 1.0);

  return percentComplete;
}

void MonitorRequest::printStatus(std::map<std::string, double>& sensorData, double time)
{
  //    bool conditionsMet = true;
  RLOG(0, "Monitor request:");
  for (size_t s = 0; s < sensorSignals.size(); s++)
  {
    double satisfied = evaluate((int) s, sensorData);

    if (satisfied >= 0.0)
    {
      double curVal = sensorData[sensorSignals[s].signal];

      RLOG(0, "%s %s: %5.2f %s %5.2f", (satisfied ? "+" : "-"),
           sensorSignals[s].signal.c_str(), curVal, (sensorSignals[s].lesserThan ? "<" : ">="),
           sensorSignals[s].threshold);
    }
  }

  RLOG(0, "Activation time: %5.2f, limit %5.2f", activationDuration, activationDurationThreshold);
}


}
