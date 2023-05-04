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

#ifndef DC_POLYGRASPDETECTOR_H
#define DC_POLYGRASPDETECTOR_H

#include <VertexArrayNode.h>
#include <Rcs_MatNd.h>
#include <Rcs_HTr.h>


namespace Dc
{
/*! \brief Class to receive polygonial 2d shapes. This class can operate in
 *         two modes:
 *
 */
class PolyGraspDetector
{
public:

  /*! \brief Constructs an instance with no polygon.
   */
  PolyGraspDetector();

  /*! \brief Destroys the members with allocated memory.
   */
  virtual ~PolyGraspDetector();

  /*! \brief This is the core function of this class. It receives a 2d
   *         polygon and computes the grasp features for it. The result is
   *         stored internally. Computed contacts and normals can be queried
   *         with the \ref getContacts() method.
   *
   *  \param[in] polygon   Array with two columns and rows corresponding to
   *                       the polygon vertices. The first and last point do
   *                       not need to match, closure is done within this
   *                       function. The polygon is assumed to be ordered
   *                       counter-clockwise.
   *  \param[in] resampleSize Distance of polygon vertices after resampling.
   *                           alues that are an order of magnitude smaller
   *                          than the hand with have turned out to be good.
   *  \param[in] halfFilter Number of polygon vertices of half the line
   *                        fitting window. The line fitting windows size is
   *                        therefore resampleSize*halfFilter*2.
   *  \param[i] maxLineFitError A line fit is performed for each vertex. This
   *                            value is decisive if a point is kept or
   *                            rejected.
   *  \param[in] handWidth Value for pruning out the grasp candidates. After
   *                       the last step of the computation, this value is
   *                       approximately the distance of neighbouring grasl
   *                       points.
   */
  void setPolygon(const MatNd* polygon,
                  double resampleSize=0.02,
                  int halfFilter=5,
                  double maxLineFitError=0.01,
                  double handWidth=0.1);// was 0.25

  /*! \brief Retrieves contact and normal results.
   *
   *  \param[out] contacts   Array of contact points (dimension nContacts x 2)
   *  \param[out] normals   Array of contact points (dimension nNormals x 2)
   */
  void getContacts(MatNd* contacts, MatNd* normals) const;

  /*! \brief Returns some debug visualization nodes.
   */
  std::vector<osg::ref_ptr<Rcs::VertexArrayNode>> createVisualizations(double normalLength, int detail);

  /*! \brief Returns some debug visualization nodes.
   */
  std::vector<osg::ref_ptr<Rcs::VertexArrayNode>> createVisualizations2(double normalLength, int detail);
  osg::Node* createVis3d(const double extrudeDir[3], double segLength=0.0);

private:

  void resamplePolygon(MatNd* polyOut,
                       const MatNd* polygon,
                       double segLength) const;

  void linefitPolygon(MatNd* polyOut,
                      MatNd* normalOut,
                      const MatNd* polygon,
                      std::vector<int>& indices,
                      int halfWindowSize,
                      double maxErr) const;

  void prunePolygon(MatNd* polyOut,
                    MatNd* normalOut,
                    const MatNd* polygon,
                    const MatNd* normals,
                    std::vector<int>& indices,
                    double neighborDist,
                    double sampleDistance) const;

  void collapseResiduals(MatNd* poly,
                         MatNd* normals,
                         std::vector<int>& indices,
                         double neighborDist) const;

  MatNd* originalPoly;
  MatNd* resampledPoly;
  MatNd* filteredPoly;
  MatNd* filteredNormals;
  MatNd* prunedPoly;
  MatNd* prunedNormals;
  MatNd* filteredNormalsVis;
  MatNd* prunedNormalsVis;
  std::vector<int> indices;
};
}

#endif   // DC_POLYGRASPDETECTOR_H
