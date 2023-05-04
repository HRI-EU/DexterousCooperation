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

#include "HoloStudyDataLogger.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_utils.h>



namespace Dc
{





HoloStudyDataLogger::HoloStudyDataLogger(EntityBase* parent) :
  ComponentBase(parent), logFile(NULL), intention(-1), motionType(-1), motionTypeStr("stopped")
{
  getEntity()->subscribe("PostUpdateGraph", &HoloStudyDataLogger::onLogData, this);
  getEntity()->subscribe("StartLogging", &HoloStudyDataLogger::onStartLogging, this);
  getEntity()->subscribe("StopLogging", &HoloStudyDataLogger::onStopLogging, this);

  getEntity()->subscribe("Intention", &HoloStudyDataLogger::onIntention, this);
  getEntity()->subscribe("MotionType", &HoloStudyDataLogger::onMotionType, this);
  getEntity()->subscribe("MotionState", &HoloStudyDataLogger::onMotionState, this);
}

HoloStudyDataLogger::~HoloStudyDataLogger()
{
  onStopLogging();
}

void HoloStudyDataLogger::logSensorState(RcsGraph* graph)
{
  fprintf(this->logFile, "%9.4f ", getEntity()->getTime());

  for (unsigned int i=0; i<graph->dof; ++i)
  {
    fprintf(this->logFile, "%f ", graph->q->ele[i]);
  }

  //  fprintf(this->logFile, "\n");
}

void HoloStudyDataLogger::logDesiredState(const MatNd* q_des)
{
  fprintf(this->logFile, "%9.4f ", getEntity()->getTime());

  for (unsigned int i=0; i<q_des->m; ++i)
  {
    fprintf(this->logFile, "%f ", q_des->ele[i]);
  }

  //  fprintf(this->logFile, "\n");

}

void HoloStudyDataLogger::onStartLogging()
{
  if (this->logFile == NULL)
  {
    char filename[256];
    File_createUniqueName(filename, "HoloStudy_", "dat");
    RLOG(0, "Logging into file \"%s\"", filename);
    this->logFile = fopen(filename, "w+");
    if (this->logFile==NULL)
    {
      RLOG(1, "Failed to open log-file \"%s\" - logging disabled", filename);
    }
  }
}

void HoloStudyDataLogger::onStopLogging()
{
  if (this->logFile != NULL)
  {
    RLOG(0, "Stopped logging into file");
    fclose(this->logFile);
    this->logFile = NULL;
  }
}

//robot moves at own speed, collaborate, don't rush, be considerate

// things to log
// 0: time
// 1-6: hand pose cur R,
// 7-12: hand pose des R
// 13: error pos R
// 14: error orientation R
// 15-20: hand_pos cur L
// 21-26: hand_pos des L
// 27: errors pos L
// 28: orientation L
// 29-34: wheel or box pose cur
// 35-40: wheel or box pose des
// 41: wheel or box pose errors pos
// 42: wheel or box pose orientation
// 43-45: head direction
// 46: intention fired
// 47: motion type

// alternative for wheel experiments:
// wheel pose cur des, errors pos, orientation

//intention and motion type are "-1" if there is none.
//I will make subscribers to set variable intention and motionType.
//intention needs to be reset to -1 after each write so each intention is only written out once
void HoloStudyDataLogger::onLogData(RcsGraph* desiredGraph, RcsGraph* currentGraph)
{
  if (this->logFile == NULL)
  {
    return;
  }

  const RcsBody* bdy_curr = NULL;
  const RcsBody* bdy_des = NULL;
  double ea[3];
  Vec3d_setZero(ea);

  // 0: time
  fprintf(this->logFile, "%f ", getEntity()->getTime());

  // 1-6: hand pose cur R
  bdy_curr = RcsGraph_getBodyByName(currentGraph, "Vicon hand right");
  RCHECK(bdy_curr);
  Mat3d_toEulerAngles(ea, (double (*)[3])bdy_curr->A_BI.rot);
  fprintf(this->logFile, "%f %f %f %f %f %f ",
          bdy_curr->A_BI.org[0], bdy_curr->A_BI.org[1], bdy_curr->A_BI.org[2], ea[0], ea[1], ea[2]);

  //  // 7-12: hand pose des R
  //  bdy_des = RcsGraph_getBodyByName(desiredGraph, "PartnerGrasp_R"); RCHECK(bdy_des);
  //  Mat3d_toEulerAngles(ea, bdy_des->A_BI->rot);
  //  fprintf(this->logFile, "%f %f %f %f %f %f ", bdy_des->A_BI->org[0], bdy_des->A_BI->org[1], bdy_des->A_BI->org[2],
  // ea[0], ea[1], ea[2]);

  //  // 13: errors pos R
  //  fprintf(this->logFile, "%f ", Vec3d_distance(bdy_curr->A_BI->org, bdy_des->A_BI->org));
  //
  //  // 14: error orientation R
  //  fprintf(this->logFile, "%f ", Mat3d_diffAngle(bdy_curr->A_BI->rot, bdy_des->A_BI->rot));

  // 15-20: hand_pos cur L
  bdy_curr = RcsGraph_getBodyByName(currentGraph, "Vicon hand left");
  RCHECK(bdy_curr);
  Mat3d_toEulerAngles(ea, (double (*)[3])bdy_curr->A_BI.rot);
  fprintf(this->logFile, "%f %f %f %f %f %f ",
          bdy_curr->A_BI.org[0], bdy_curr->A_BI.org[1], bdy_curr->A_BI.org[2], ea[0], ea[1], ea[2]);

  //  // 21-26: hand_pos des L
  //  bdy_des = RcsGraph_getBodyByName(desiredGraph, "PartnerGrasp_L"); RCHECK(bdy_des);
  //  Mat3d_toEulerAngles(ea, bdy_des->A_BI->rot);
  //  fprintf(this->logFile, "%f %f %f %f %f %f ", bdy_des->A_BI.org[0], bdy_des->A_BI.org[1], bdy_des->A_BI.org[2],
  //ea[0], ea[1], ea[2]);

  //  // 27: errors pos L
  //  fprintf(this->logFile, "%f ", Vec3d_distance(bdy_curr->A_BI.org, bdy_des->A_BI.org));
  //
  //  // 28: error orientation L
  //  fprintf(this->logFile, "%f ", Mat3d_diffAngle(bdy_curr->A_BI->rot, bdy_des->A_BI->rot));

  // 29-34: wheel or box pose cur
  bdy_curr = RcsGraph_getBodyByName(currentGraph, "Vicon box");
  if (bdy_curr==NULL)
  {
    bdy_curr = RcsGraph_getBodyByName(currentGraph, "Vicon tire");
  }
  RCHECK(bdy_curr);
  Mat3d_toEulerAngles(ea, (double (*)[3])bdy_curr->A_BI.rot);
  fprintf(this->logFile, "%f %f %f %f %f %f ", bdy_curr->A_BI.org[0], bdy_curr->A_BI.org[1],
          bdy_curr->A_BI.org[2], ea[0], ea[1], ea[2]);

  // 35-40: wheel or box pose des
  bdy_des = RcsGraph_getBodyByName(desiredGraph, "Box_v");
  if (bdy_des==NULL)
  {
    bdy_des = RcsGraph_getBodyByName(desiredGraph, "Wheel");
  }
  Mat3d_toEulerAngles(ea, (double (*)[3])bdy_des->A_BI.rot);
  fprintf(this->logFile, "%f %f %f %f %f %f ", bdy_des->A_BI.org[0], bdy_des->A_BI.org[1],
          bdy_des->A_BI.org[2], ea[0], ea[1], ea[2]);

  // 41: wheel or box pose errors pos
  fprintf(this->logFile, "%f ", Vec3d_distance(bdy_curr->A_BI.org, bdy_des->A_BI.org));

  // 42: wheel or box pose orientation
  fprintf(this->logFile, "%f ", Mat3d_diffAngle((double (*)[3])bdy_curr->A_BI.rot, (double (*)[3])bdy_des->A_BI.rot));

  // 43-45: head direction
  bdy_curr = RcsGraph_getBodyByName(currentGraph, "Hololens_SlamFrame");
  RCHECK(bdy_curr);
  Mat3d_toEulerAngles(ea, (double (*)[3])bdy_curr->A_BI.rot);
  fprintf(this->logFile, "%f %f %f %f %f %f ", bdy_curr->A_BI.org[0], bdy_curr->A_BI.org[1],
          bdy_curr->A_BI.org[2], ea[0], ea[1], ea[2]);

  // 46: intention fired
  fprintf(this->logFile, "%d ", intention);

  // 47: motion type
  fprintf(this->logFile, "%d ", motionType);

  logSensorState(currentGraph);

  logDesiredState(desiredGraph->q);

  // Final end of line
  fprintf(this->logFile, "\n");
}


void HoloStudyDataLogger::onIntention(int intention_id, int seq)
{
  RLOG(0, "Received intention %d with seq %d", intention_id, seq);

  intention = intention_id;
}

void HoloStudyDataLogger::onMotionType(int type, std::string transitionStr)
{
  RLOG(0, "Received motionType %d (%s)", type, transitionStr.c_str());

  motionType = type;
  motionTypeStr = transitionStr;
}

void HoloStudyDataLogger::onMotionState(bool isMoving)
{
  //TODO: motionstate needs to be defined in a better place!

  if (isMoving == false)
  {
    RLOG(0, "Motion Stopped resetting type: %d -> -1", motionType);

    motionType = -1;
    motionTypeStr = "stopped";
  }
}

}  // namespace
