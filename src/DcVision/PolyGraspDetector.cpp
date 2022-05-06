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

#include "PolyGraspDetector.h"

#include <Rcs_geometry.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>

#include <RcsViewer.h>
#include <ExtrudedPolygonNode.h>
#include <SphereNode.h>
#include <osgText/Text>

#include <cmath>
#include <numeric>



/*******************************************************************************
 * Circulat modulo
 ******************************************************************************/
static inline int modState(int stateValue, int x)
{
  int result = stateValue%x;

  if (result < 0)
  {
    result += x;
  }

  return result;
}

static bool pruneGraspPoints_(MatNd* graspPoints,
                              MatNd* graspNormals,
                              std::vector<int>& indices,
                              double dThres,
                              const MatNd* resampledPoly,
                              bool reversed)
{
  double (*poly)[2] = (double (*)[2])resampledPoly->ele;

  for (unsigned int i_=0; i_<graspPoints->m; ++i_)
  {
    int i = i_;
    if (reversed)
    {
      i = graspPoints->m - i_ - 1;
    }


    const int idx_minus = indices[modState(i-1, graspPoints->m)];
    const int idx = indices[modState(i, graspPoints->m)];
    const int idx_plus = indices[modState(i+1, graspPoints->m)];

    double dist_minus = Math_polyVertexDistance(poly, resampledPoly->m,
                                                idx_minus, idx);
    double dist_plus  = Math_polyVertexDistance(poly, resampledPoly->m,
                                                idx, idx_plus);

    if ((dist_minus<dThres) && (dist_plus<dThres))
    {
      NLOG(0, "%s", reversed ? "REVERSED" : "NOT REVERSED");
      indices.erase(indices.begin()+i);
      MatNd_deleteRow(graspPoints, i);
      MatNd_deleteRow(graspNormals, i);
      return true;
    }

  }

  NLOG(0, "%s", reversed ? "REVERSED" : "NOT REVERSED");

  return false;
}

static bool pruneGraspPoints2_(MatNd* graspPoints,
                               MatNd* normals,
                               std::vector<int>& indices,
                               double dThres,
                               const MatNd* resampledPoly)
{
  double (*poly)[2] = (double (*)[2])resampledPoly->ele;

  for (unsigned int i=0; i<graspPoints->m; ++i)
  {
    const int idx_minus = indices[modState(i-1, graspPoints->m)];
    const int idx = indices[modState(i, graspPoints->m)];
    const int idx_plus = indices[modState(i+1, graspPoints->m)];

    double dist_minus = Math_polyVertexDistance(poly, resampledPoly->m,
                                                idx_minus, idx);
    double dist_plus  = Math_polyVertexDistance(poly, resampledPoly->m,
                                                idx, idx_plus);

    if ((dist_minus<dThres) && (dist_plus<dThres))
    {
      indices.erase(indices.begin()+i);
      MatNd_deleteRow(graspPoints, i);
      MatNd_deleteRow(normals, i);
      return true;
    }
    else if ((dist_minus<dThres) && (dist_plus>=dThres))   // Merge -1 , 0
    {
      indices.erase(indices.begin()+i);
      MatNd_deleteRow(graspPoints, i);
      MatNd_deleteRow(normals, i);
      return true;
    }
    else if ((dist_minus>=dThres) && (dist_plus<dThres))   // Merge 0 , 1
    {
      indices.erase(indices.begin()+i);
      MatNd_deleteRow(graspPoints, i);
      MatNd_deleteRow(normals, i);
      return true;
    }

  }

  return false;
}



namespace Rcs
{

PolyGraspDetector::PolyGraspDetector() : originalPoly(NULL),
  resampledPoly(NULL),
  filteredPoly(NULL),
  filteredNormals(NULL),
  prunedPoly(NULL),
  prunedNormals(NULL),
  filteredNormalsVis(NULL),
  prunedNormalsVis(NULL)
{
}

PolyGraspDetector::~PolyGraspDetector()
{
  MatNd_destroy(this->originalPoly);
  MatNd_destroy(this->resampledPoly);
  MatNd_destroy(this->filteredPoly);
  MatNd_destroy(this->filteredNormals);
  MatNd_destroy(this->filteredNormalsVis);
  MatNd_destroy(this->prunedPoly);
  MatNd_destroy(this->prunedNormals);
  MatNd_destroy(this->prunedNormalsVis);
}

void PolyGraspDetector::setPolygon(const MatNd* polygon,
                                   double resampleSize,
                                   int halfFilter,
                                   double maxLineFitError,
                                   double handWidth)
{
  if (polygon==NULL)
  {
    RLOG(0, "Can't process NULL polygon - skipping");
    return;
  }

  MatNd_destroy(this->originalPoly);
  this->originalPoly = MatNd_clone(polygon);

  double (*poly)[2] = (double (*)[2])originalPoly->ele;
  bool clockwise = Math_isPolygonClockwise(poly, originalPoly->m);
  if (clockwise)
  {
    RLOG(5, "Reversing polygon due to clockwise winding order");
    MatNd_reverseSelf(originalPoly);
    clockwise = Math_isPolygonClockwise(poly, originalPoly->m);
    RCHECK(clockwise==false);
  }

  this->resampledPoly = MatNd_realloc(resampledPoly, polygon->m, polygon->n);
  this->filteredPoly = MatNd_realloc(filteredPoly, polygon->m, polygon->n);
  this->filteredNormals = MatNd_realloc(filteredNormals, polygon->m, polygon->n);
  this->filteredNormalsVis = MatNd_realloc(filteredNormalsVis, 2*polygon->m, polygon->n);
  this->prunedPoly = MatNd_realloc(prunedPoly, polygon->m, polygon->n);
  this->prunedNormals = MatNd_realloc(prunedNormals, polygon->m, polygon->n);
  this->prunedNormalsVis = MatNd_realloc(prunedNormalsVis, 2*polygon->m, polygon->n);

  RLOG(5, "resamplePolygon with resampleSize = %f", resampleSize);
  resamplePolygon(resampledPoly, originalPoly, resampleSize);

  indices.resize(resampledPoly->m);
  // Fill with 0, 1, ..., resampledPoly->m-1
  std::iota(std::begin(indices), std::end(indices), 0);
  RLOG(5, "linefitPolygon");
  linefitPolygon(filteredPoly, filteredNormals, resampledPoly, indices,
                 halfFilter, maxLineFitError);

  RLOG(5, "prunePolygon");
  prunePolygon(prunedPoly, prunedNormals, filteredPoly, filteredNormals,
               indices, handWidth, resampleSize);

  RLOG(5, "collapseResiduals");
  collapseResiduals(prunedPoly, prunedNormals, indices, handWidth);
}

void PolyGraspDetector::getContacts(MatNd* contacts, MatNd* normals) const
{
  MatNd_resizeCopy(&contacts, this->prunedPoly);
  MatNd_resizeCopy(&normals, this->prunedNormals);
}

void PolyGraspDetector::resamplePolygon(MatNd* polyOut,
                                        const MatNd* polygon,
                                        double segLength) const
{
  RCHECK(polygon->n==2);
  RLOG(5, "Original polygon has %d vertices", polygon->m);
  //MatNd_printCommentDigits("polygon", polygon, 4);
  double (*poly)[2] = (double (*)[2])polygon->ele;
  const unsigned int nVertices = polygon->m;
  const double len = Math_lengthPolygon2D(poly, nVertices);
  const unsigned int nvOut = lround(len/segLength);
  RLOG(5, "Poly length = %f with %d vertices", len, nvOut);
  MatNd_realloc(polyOut, nvOut, 2);
  double (*polyR)[2] = (double (*)[2])polyOut->ele;
  Math_resamplePolygon2D(polyR, nvOut, poly, nVertices);
}

void PolyGraspDetector::linefitPolygon(MatNd* polyOut,
                                       MatNd* normalOut,
                                       const MatNd* polygon,
                                       std::vector<int>& indices_,
                                       int halfWindowSize,
                                       double maxErr) const
{
  std::vector<int> indices;

  const unsigned int nVertices = polygon->m;
  MatNd_realloc(polyOut, nVertices, 2);
  MatNd_realloc(normalOut, nVertices, 2);
  const int windowSize = 2*halfWindowSize-1;
  RCHECK(windowSize>0);
  unsigned int nGraspPoints = 0;
  MatNd* window = MatNd_create(windowSize, 2);
  double (*poly)[2] = (double (*)[2])polygon->ele;

  for (int i = 0; i < (int) nVertices; ++i)
  {
    // Calculate line fit on window
    const int widx0 = i-halfWindowSize+1;
    const int widx1 = i+halfWindowSize;
    for (int j = widx0; j<widx1; ++j)
    {
      double* row_i = MatNd_getRowPtr(window, j-widx0);
      const int idx = modState(j, nVertices);
      row_i[0] = poly[idx][0];
      row_i[1] = poly[idx][1];
      RLOG(10, "j=%d idx=%d", j, idx);
    }

    // Add a bit of noise to avoid no solution for vertical slope of A
    //VecNd_addRandom(window->ele, -1.0e-5, 1.0e-5, window->m*window->n);
    double A=0.0, B=0.0;
    bool lineFitOk = MatNd_lineFit2D(&A, &B, window);
    if (!lineFitOk)
    {
      MatNd_printCommentDigits("window", window, 5);
    }
    RCHECK(lineFitOk);

    // line fit: y = Ax + B
    // x1=0 y1=B
    // x2=1 y2=A+B
    // dx = 1   dy = A
    // normals: (-A , 1) and (A , -1)
    // Counter-clockwise vertex ordering: normal is (A , -1)

    double segPt[3], segDir[3];
    Vec3d_set(segPt, 0.0, B, 0.0);
    Vec3d_set(segDir, 1.0, A, 0.0);

    // Calculate point distances
    // y = Ax + B:
    // x=0: y = B => (0 B 0) is line point
    //      y' = A => (1 A 0) is line dir
    double err = 0.0;
    for (int j = i-halfWindowSize+1; j<i+halfWindowSize; ++j)
    {
      RCHECK(j-(i-halfWindowSize+1)>=0);
      double* row_i = MatNd_getRowPtr(window, j-(i-halfWindowSize+1));
      double pt[3], closest[3];
      Vec3d_set(pt, row_i[0], row_i[1], 0.0);
      err = fmax(err, Math_sqrDistPointLine(pt, segPt, segDir, closest));
    }

    err = sqrt(err);

    if (err < maxErr)
    {
      indices.push_back(indices_[i]);
      MatNd_set(polyOut, nGraspPoints, 0, poly[i][0]);
      MatNd_set(polyOut, nGraspPoints, 1, poly[i][1]);

      // That's the normal on the line fit. There's two solutions, one pointing
      // to the inside of the polygon, and one pointing outwards. We determine
      // the outward pointing normal with one of the two solutions below.
      double n[2];
      n[0] = A;
      n[1] = -1.0;

      // Alternative check: cross product. It is much faster and more robust.
      // Tangent direction: (1 A) normal direction: (A -1) and (-A 1)
      // normal x tangent must point up.
      // Change sign if A is in oposite direction to consecutive vertices
      double windowDir[2], tangentDir[2];
      windowDir[0] = MatNd_get(window, window->m-1, 0) - MatNd_get(window, 0, 0);
      windowDir[1] = MatNd_get(window, window->m-1, 1) - MatNd_get(window, 0, 1);
      tangentDir[0] = 1.0;
      tangentDir[1] = A;
      VecNd_normalizeSelf(windowDir, 2);
      VecNd_normalizeSelf(tangentDir, 2);

      double wSign = windowDir[0]*tangentDir[0] + windowDir[1]*tangentDir[1];
      if (wSign < 0.0)
      {
        n[0] = -n[0];
        n[1] = -n[1];
      }

      VecNd_normalizeSelf(n, 2);

      MatNd_set(normalOut, nGraspPoints, 0, n[0]);
      MatNd_set(normalOut, nGraspPoints, 1, n[1]);

      nGraspPoints++;
    }
    else
    {
      RLOG(5, "Skipping data point %d with error %g: A=%g B=%g", i, err, A, B);

      REXEC(5)
      {
        for (int j = i-halfWindowSize+1; j<i+halfWindowSize; ++j)
        {
          double* row_i = MatNd_getRowPtr(window, j-(i-halfWindowSize+1));
          double pt[3], closest[3];
          Vec3d_set(pt, row_i[0], row_i[1], 0.0);
          double d = Math_sqrDistPointLine(pt, segPt, segDir, closest);
          RLOG(0, "%d: err(%d) = %f", i, j-(i-halfWindowSize+1), sqrt(d));
        }
      }

    }

  }   // for (int i = 0; i < (int) nVertices; ++i)

  MatNd_reshape(polyOut, nGraspPoints, 2);
  MatNd_reshape(normalOut, nGraspPoints, 2);
  MatNd_destroy(window);

  indices_ = indices;
}

void PolyGraspDetector::prunePolygon(MatNd* polyOut,
                                     MatNd* normalOut,
                                     const MatNd* polygon,
                                     const MatNd* normals,
                                     std::vector<int>& indices,
                                     double neighborDist,
                                     double sampleDistance) const
{
  MatNd_realloc(polyOut, polygon->m, polygon->n);
  MatNd_realloc(normalOut, normals->m, normals->n);
  MatNd_copy(polyOut, polygon);
  MatNd_copy(normalOut, normals);

  int iterations = lround(ceil(neighborDist/sampleDistance));
  size_t deleted = 0;
  //RLOG(0, "iterations: %d", iterations);

  for (int i=1; i<iterations; ++i)
  {
    //  RLOG(0, "iteration %d", i);
    double scaling = 0.5*((double)i+1.0)/iterations;

    while (pruneGraspPoints_(polyOut, normalOut, indices, scaling*neighborDist,
                             resampledPoly, deleted%2==0))
    {
      deleted++;
    }
    RLOG_CPP(10, "Deleted " << deleted << " points for run " << i
             << " with scaling " << scaling << " and hand width "
             << scaling*neighborDist);

  }

  return;
}

void PolyGraspDetector::collapseResiduals(MatNd* poly,
                                          MatNd* normals,
                                          std::vector<int>& indices,
                                          double neighborDist) const
{
  size_t deleted = 0;
  while (pruneGraspPoints2_(poly, normals, indices, 0.25*neighborDist,
                            resampledPoly))
  {
    deleted++;
  }

  RLOG_CPP(10, "Deleted " << deleted << " points for final run");
}

std::vector<osg::ref_ptr<Rcs::VertexArrayNode>>
                                             PolyGraspDetector::createVisualizations(double normalLength, int detail)
{
  std::vector<osg::ref_ptr<Rcs::VertexArrayNode>> nodes;

  if (originalPoly==NULL)
  {
    return nodes;
  }

  // Visualize original polygon as line loop
  osg::ref_ptr<Rcs::VertexArrayNode> vn1;
  vn1 = new Rcs::VertexArrayNode(osg::PrimitiveSet::LINE_LOOP);
  vn1->copyPoints(originalPoly);
  vn1->setLighting(false);

  // Visualize original polygon as points
  osg::ref_ptr<Rcs::VertexArrayNode> vn2;
  vn2 = new Rcs::VertexArrayNode(osg::PrimitiveSet::POINTS);
  vn2->copyPoints(originalPoly);
  vn2->setPointSize(15);
  vn2->setLighting(false);

  // Visualize re-sampled polygon as points
  osg::ref_ptr<Rcs::VertexArrayNode> vn3;
  vn3 = new Rcs::VertexArrayNode(osg::PrimitiveSet::POINTS);
  vn3->copyPoints(resampledPoly);
  vn3->setPointSize(25);
  vn3->setMaterial("YELLOW");
  vn3->setPosition(0.0, 0.0, 0.1);
  vn3->setLighting(false);

  // Visualize line-fitted grasp points
  osg::ref_ptr<Rcs::VertexArrayNode> vn4;
  vn4 = new Rcs::VertexArrayNode(osg::PrimitiveSet::POINTS);
  vn4->copyPoints(filteredPoly);
  vn4->setPointSize(30);
  vn4->setPosition(0.0, 0.0, 0.2);
  vn4->setLighting(false);
  vn4->setMaterial("BLUE");

  // Visualize line-fitted grasp normals
  this->filteredNormalsVis = MatNd_realloc(filteredNormalsVis,
                                           2*filteredNormals->m, 2);

  for (unsigned int i=0; i<filteredNormals->m; ++i)
  {
    double* row0 = MatNd_getRowPtr(filteredNormalsVis, i*2+0);
    double* row1 = MatNd_getRowPtr(filteredNormalsVis, i*2+1);
    row0[0] = MatNd_get(filteredPoly, i, 0);
    row0[1] = MatNd_get(filteredPoly, i, 1);
    row1[0] = row0[0] + normalLength*MatNd_get(filteredNormals, i, 0);
    row1[1] = row0[1] + normalLength*MatNd_get(filteredNormals, i, 1);
  }
  Rcs::VertexArrayNode* vn5;
  vn5 = new Rcs::VertexArrayNode(osg::PrimitiveSet::LINES);
  vn5->copyPoints(filteredNormalsVis);
  vn5->setPosition(0.0, 0.0, 0.2);
  vn5->setLighting(false);
  vn5->setMaterial("BLUE");
  vn4->addChild(vn5);

  // Visualize pruned grasp points
  osg::ref_ptr<Rcs::VertexArrayNode> vn6;
  vn6 = new Rcs::VertexArrayNode(osg::PrimitiveSet::POINTS);
  // vn6->setPoints(prunedPoly);
  // vn6->setPointSize(30);
  // vn6->setPosition(0.0, 0.0, 0.3);
  // vn6->setLighting(false);

  // Add text labels to the grasp points
  for (size_t i=0; i<prunedPoly->m; ++i)
  {
    const double* posPtr = MatNd_getRowPtr(prunedPoly, i);
    double pos[3];
    Vec3d_set(pos, posPtr[0], posPtr[1], 0.0);   // prunedPoly has 2 columns
    osg::ref_ptr<SphereNode> cn = new SphereNode(pos, 0.01);
    cn->setMaterial("RED");

    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode();
    cn->addChild(textGeode.get());
    osg::ref_ptr<osgText::Text> text = new osgText::Text();
    text->setCharacterSize(0.05);
    text->setText(std::to_string(i));
    text->setAlignment(osgText::Text::LEFT_CENTER);
    text->setAxisAlignment(osgText::Text::SCREEN);
    char fontFile[256] = "";
    bool fontFound = Rcs_getAbsoluteFileName("fonts/VeraMono.ttf", fontFile);
    if (fontFound)
    {
      text->setFont(fontFile);
    }
    //text->setColor(colorFromString("RED"));
    textGeode->addDrawable(text.get());

    vn6->addChild(cn.get());
  }

  // // Visualize pruned grasp normals
  this->prunedNormalsVis = MatNd_realloc(prunedNormalsVis,
                                         2*prunedNormals->m, 2);
  for (unsigned int i=0; i<prunedNormals->m; ++i)
  {
    double* row0 = MatNd_getRowPtr(prunedNormalsVis, i*2+0);
    double* row1 = MatNd_getRowPtr(prunedNormalsVis, i*2+1);
    row0[0] = MatNd_get(prunedPoly, i, 0);
    row0[1] = MatNd_get(prunedPoly, i, 1);
    row1[0] = row0[0] + normalLength*MatNd_get(prunedNormals, i, 0);
    row1[1] = row0[1] + normalLength*MatNd_get(prunedNormals, i, 1);
  }
  Rcs::VertexArrayNode* vn7;
  vn7 = new Rcs::VertexArrayNode(osg::PrimitiveSet::LINES);
  vn7->copyPoints(prunedNormalsVis);
  vn7->setPosition(0.0, 0.0, 0.3);
  vn7->setLighting(false);
  vn6->addChild(vn7);

  if (detail == 0)
  {
    vn6->setPosition(0.0, 0.0, 0.0);
    vn7->setPosition(0.0, 0.0, 0.0);
    nodes.push_back(vn1);   // original polygon as line loop
    nodes.push_back(vn6);   // pruned grasp points
    //nodes.push_back(vn7);   // pruned grasp normals
  }
  else if (detail == 1)
  {
    nodes.push_back(vn1);   // original polygon as line loop
    //nodes.push_back(vn2);   // original polygon as points
    nodes.push_back(vn3);   // re-sampled polygon as points
    nodes.push_back(vn4);   // line-fitted grasp points
    //nodes.push_back(vn5);   // line-fitted grasp normals
    nodes.push_back(vn6);   // pruned grasp points
    //nodes.push_back(vn7);   // pruned grasp normals
  }
  else if (detail == 2)
  {
    vn1->setPosition(0.0, 0.0, 0.3);
    //vn2->setPosition(0.0, 0.0, 0.3);
    vn6->addChild(vn1);

    nodes.push_back(vn2);   // original polygon as line loop
    //nodes.push_back(vn2);   // original polygon as points
    nodes.push_back(vn3);   // re-sampled polygon as points
    nodes.push_back(vn4);   // line-fitted grasp points
    //nodes.push_back(vn5);   // line-fitted grasp normals
    nodes.push_back(vn6);   // pruned grasp points
    //nodes.push_back(vn7);   // pruned grasp normals
  }

  RCHECK_EQ(indices.size(), prunedPoly->m);

  return nodes;
}


osg::Node* PolyGraspDetector::createVis3d(const double extrudeDir[3],
                                          double segLength)
{
  osg::Node* extrusion = NULL;

  if (segLength==0.0)
  {
    extrusion = new ExtrudedPolygonNode(originalPoly, extrudeDir);
  }
  else
  {
    MatNd* coarsePoly = MatNd_create(1, 2);
    resamplePolygon(coarsePoly, originalPoly, segLength);
    extrusion = new ExtrudedPolygonNode(coarsePoly, extrudeDir);
    MatNd_destroy(coarsePoly);
  }

  return extrusion;
}




std::vector<osg::ref_ptr<Rcs::VertexArrayNode>>
                                             PolyGraspDetector::createVisualizations2(double normalLength, int detail)
{
  std::vector<osg::ref_ptr<Rcs::VertexArrayNode>> nodes;

  if (originalPoly==NULL)
  {
    return nodes;
  }

  RCHECK_EQ(prunedPoly->n, 2);

  // Visualize original polygon as line loop
  osg::ref_ptr<Rcs::VertexArrayNode> vn1;
  vn1 = new Rcs::VertexArrayNode(osg::PrimitiveSet::LINE_LOOP);
  vn1->copyPoints(originalPoly);
  vn1->setLighting(false);

  // Add text labels to the grasp points
  for (size_t i=0; i<prunedPoly->m; ++i)
  {
    const double* posPtr = MatNd_getRowPtr(prunedPoly, i);
    double pos[3];
    Vec3d_set(pos, posPtr[0], posPtr[1], 0.0);   // prunedPoly has 2 columns
    RLOG(0, "2d poly is %f %f", posPtr[0], posPtr[1]);
    osg::ref_ptr<SphereNode> cn = new SphereNode(pos, 0.01);
    cn->setMaterial("RED");

    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode();
    cn->addChild(textGeode.get());
    osg::ref_ptr<osgText::Text> text = new osgText::Text();
    text->setCharacterSize(0.05);
    text->setText(std::to_string(i));
    text->setAlignment(osgText::Text::LEFT_CENTER);
    text->setAxisAlignment(osgText::Text::SCREEN);
    char fontFile[256] = "";
    bool fontFound = Rcs_getAbsoluteFileName("fonts/VeraMono.ttf", fontFile);
    if (fontFound)
    {
      text->setFont(fontFile);
    }
    //text->setColor(colorFromString("RED"));
    textGeode->addDrawable(text.get());

    vn1->addChild(cn.get());
  }


  nodes.push_back(vn1);

  RCHECK_EQ(indices.size(), prunedPoly->m);

  return nodes;
}

}   // namespace Rcs
