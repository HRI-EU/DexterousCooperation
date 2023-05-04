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

#include "PolyROSComponent.h"
#include "PolygonObjectModel.h"
#include "EntityBase.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_quaternion.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>

#define WITH_GRAPHICS

#if defined (WITH_GRAPHICS)
#include <SphereNode.h>
#include <BoxNode.h>
#include <CylinderNode.h>
#include <VertexArrayNode.h>
#include <COSNode.h>
#include <TextNode3D.h>
#include <osg/MatrixTransform>
#endif

#include <thread>


namespace Dc
{

PolyROSComponent::PolyROSComponent(EntityBase* parent, std::string rosTopic,
                                   bool polyDebug) :
  ComponentBase(parent),
  ntt(getEntity()),
  graspDetector(),
  rosFullTopicName(rosTopic),
  graphicsNodeName("PolyROSObjectChild"),
  debugMode(polyDebug),
  listenToROS(true),
  viewFromKinect(false),
  toggleViewFromKinect(false),
  T_camI(NULL),
  T_objI(NULL),
  T_polyI(NULL)
{
  initMtx.lock();
  objectBdy = "Box_v";

  if (debugMode==false)
  {
    // Subscribe to ROS
    subscribe("Start", &PolyROSComponent::onStart);
    subscribe("Stop", &PolyROSComponent::onStop);
    subscribe("GetPolygonsFromROS", &PolyROSComponent::onListenToROS);
  }
  else
  {
    // Subscribe to ECS event that loads polygon files.
    subscribe<std::string>("ReloadPolygon", &PolyROSComponent::onSetNewData);
  }

  subscribe("PostUpdateGraph", &PolyROSComponent::onPostUpdateGraph);
  subscribe("SetViewFromKinect", &PolyROSComponent::onToggleViewFromKinect);
}

PolyROSComponent::~PolyROSComponent()
{
  // Calling free on NULL does nothing
  RFREE(this->T_camI);
  RFREE(this->T_objI);
  RFREE(this->T_polyI);
}

void PolyROSComponent::setCameraBodyName(const std::string& camName)
{
  this->cameraBdy = camName;
}

void PolyROSComponent::updateObjectFrame(RcsGraph* graph, const RcsBody* obj)
{
  HTr A_CI;

  trfMtx.lock();
  if (this->T_polyI==NULL)
  {
    trfMtx.unlock();
    return;
  }
  HTr_copy(&A_CI, this->T_polyI);
  RFREE(this->T_polyI);
  this->T_polyI = NULL;
  trfMtx.unlock();

  HTr A_CP;
  const RcsBody* parent = RCSBODY_BY_ID(graph, obj->parentId);
  const HTr* A_parentI = parent ? &parent->A_BI : HTr_identity();
  HTr_invTransform(&A_CP, A_parentI, &A_CI);
  //const RcsJoint* objJnt = RCSJOINT_BY_ID(graph, obj->jntId);
  const RcsJoint* objJnt = &graph->joints[obj->jntId];
  double* q_obj = &graph->q->ele[objJnt->jointIndex];
  HTr_to6DVector(q_obj, &A_CP);

  getEntity()->publish<const RcsGraph*>("InitFromState", graph);
}

void PolyROSComponent::onToggleViewFromKinect()
{
  viewFromKinect = !viewFromKinect;
  toggleViewFromKinect = true;
}

void PolyROSComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  // If no camera name has been assigned, there was no polygon received. We
  // therefore don't update the transforms for either camera or object.
  trfMtx.lock();
  std::string camBdy = this->cameraBdy;
  trfMtx.unlock();

  if (camBdy.empty())
  {
    RLOG(6, "No camera body assigned");
    return;
  }

  const RcsBody* cam = RcsGraph_getBodyByName(desired, camBdy.c_str());

  if (cam==NULL)
  {
    RLOG(0, "Couldn't find camera reference \"%s\" in graph", camBdy.c_str());
    return;
  }

  trfMtx.lock();
  if (T_camI == NULL)
  {
    this->T_camI = HTr_clone(&cam->A_BI);
  }
  else
  {
    HTr_copy(this->T_camI, &cam->A_BI);
  }
  trfMtx.unlock();

  const RcsBody* obj = RcsGraph_getBodyByName(desired, objectBdy.c_str());

  if (obj==NULL)
  {
    RLOG(0, "Couldn't find object \"%s\" in graph", objectBdy.c_str());
    return;
  }

  if (RcsBody_isFloatingBase(desired, obj)==false)
  {
    RLOG(0, "Object \"%s\" has no rigid body degrees of freedom",
         objectBdy.c_str());
    return;
  }

  // If a new polygon transform has been computed, this function changes the
  // rigid body degrees of freedom of the objectBody so that it matches the
  // new transform. In case of a change, the function publishes the
  // "InitFromState" event that updates all components with the modified
  // transformation.
  updateObjectFrame(desired, obj);

  trfMtx.lock();
  if (T_objI == NULL)
  {
    T_objI = HTr_clone(&obj->A_BI);
    initMtx.unlock();
  }
  else
  {
    HTr_copy(T_objI, &obj->A_BI);
  }

  trfMtx.unlock();

  // Handle the change of the viewer's perspective between mouse manipulator
  // and fixed to the Kinect's transform. In case the view is set to the
  // Kinect, we update the camera transform.
  handleViewChange(&cam->A_BI);
}

// Change viewer to camera perspective
// der FoV der Kinect2 ist circa:
// RGB: 84 x 54 deg (1920 x 1080)
// IR/D: 71 x 60deg  (512 x 424)
void PolyROSComponent::handleViewChange(const HTr* A_camI)
{
  if (viewFromKinect)
  {
    HTr camTrf;
    HTr_copy(&camTrf, A_camI);
    Mat3d_rotateSelfAboutXYZAxis(camTrf.rot, 1, -M_PI_2);
    Mat3d_rotateSelfAboutXYZAxis(camTrf.rot, 0, M_PI_2);
    char trfStr[256];
    HTr_toString(trfStr, &camTrf);
    ntt->publish("RenderCommand", std::string("CameraTransform"),
                 std::string(trfStr));

    if (toggleViewFromKinect)
    {
      toggleViewFromKinect = false;
      ntt->publish("RenderCommand", std::string("FieldOfView"),
                   std::string("71 71"));   // was "71 60"
    }
  }
  else
  {
    if (toggleViewFromKinect)
    {
      toggleViewFromKinect = false;
      ntt->publish("RenderCommand", std::string("ResetView"),
                   std::string("30 40"));
    }

  }

}

MatNd* PolyROSComponent::computePolyFrame(const geometry_msgs::PolygonStamped::ConstPtr& rosPoly, HTr* A_polyWorld)
{
  HTr A_camI;

  trfMtx.lock();
  if (T_camI==NULL)
  {
    trfMtx.unlock();
    RLOG(1, "Camera and body transforms not found");
    return NULL;
  }

  HTr_copy(&A_camI, T_camI);
  trfMtx.unlock();

  //////////////////////////////////////////////////////////
  // Convert ROS polygon to tmpVerts (in camera
  // coordinates), units from ROS are in [mm]
  //////////////////////////////////////////////////////////
  size_t nVertices = rosPoly->polygon.points.size();

  if (nVertices < 3)
  {
    RLOG_CPP(1, "Can't update polygin with less than 3 vertices, this one has"
             << nVertices);
    return NULL;
  }

  geometry_msgs::Point32 firstPt = rosPoly->polygon.points[0];
  geometry_msgs::Point32 lastPt = rosPoly->polygon.points[nVertices-1];
  if ((lastPt.x==firstPt.x) && (lastPt.y==firstPt.y) && (lastPt.z==firstPt.z))
  {
    RLOG(5, "First and last point coincide - ignoring last point");
    nVertices--;
  }

  MatNd* tmpVerts = MatNd_create(nVertices, 3);

  for (size_t i=0; i<nVertices; ++i)
  {
    double* row_i = MatNd_getRowPtr(tmpVerts, i);
    row_i[0] = rosPoly->polygon.points[i].x;
    row_i[1] = rosPoly->polygon.points[i].y;
    row_i[2] = rosPoly->polygon.points[i].z;
  }


  //////////////////////////////////////////////////////////
  // Write out poly file as long as it is in camera
  // coordinates. We only do this when connected to ROS,
  // and not in the debug version of the class. Otherwise
  // we would overwrite the polygons to be debugged.
  //////////////////////////////////////////////////////////
  if (debugMode==false)
  {
    static int foutCount = 0;
    std::string foutName = "poly_" + std::to_string(foutCount) + ".dat";
    foutCount++;
    MatNd_toFile(tmpVerts, foutName.c_str());
  }


  //////////////////////////////////////////////////////////
  // Convert positions to meter and transform into world
  // coordinates
  //////////////////////////////////////////////////////////
  for (size_t i=0; i<nVertices; ++i)
  {
    double* row_i = MatNd_getRowPtr(tmpVerts, i);
    Vec3d_constMulSelf(row_i, 0.001);
    Vec3d_transformSelf(row_i, &A_camI);
  }


  //////////////////////////////////////////////////////////
  // Eigen-decomposition, in world coordinates.
  //////////////////////////////////////////////////////////
  double centroid[3];

  for (int i=0; i<3; ++i)
  {
    centroid[i] = MatNd_columnSum(tmpVerts, i);
  }
  Vec3d_constMulSelf(centroid, 1.0/tmpVerts->m);

  for (unsigned int i=0; i<tmpVerts->m; ++i)
  {
    double* row_i = MatNd_getRowPtr(tmpVerts, i);
    Vec3d_subSelf(row_i, centroid);
  }


  MatNd* eig = MatNd_create(3,3);
  MatNd_dyadicProduct(eig, tmpVerts);
  double V[3][3], d[3];
  bool success = Mat3d_getEigenVectors(V, d, (double (*)[3])eig->ele);
  MatNd_destroy(eig);

  if (success==false)
  {
    RLOG(1, "Failed to compute polygon Eigenbasis");
    MatNd_destroy(tmpVerts);
    return NULL;
  }

  double dir[3];
  int minEig = VecNd_indexMin(d, 3);

  for (int i=0; i<3; ++i)
  {
    dir[i] = V[i][minEig];
  }


  //////////////////////////////////////////////////////////
  // Normal vector should oppose camera view direction. We
  // are looking towards the surface the normal vector is
  // perpendicular to.
  //////////////////////////////////////////////////////////
  double ang = Vec3d_diffAngle(dir, A_camI.rot[2]);
  if (ang < M_PI_2)
  {
    Vec3d_constMulSelf(dir, -1.0);
  }


  //////////////////////////////////////////////////////////
  // Compute polygon frame in centroidal coordinates. Here
  // is how we do it:
  // z-axis points along the plane's Eigenvector that is
  //   normal to the plane and directed towards the camera
  // x-axis lies in the horizondal plane
  // y-axis is right-handed to both x any z
  //////////////////////////////////////////////////////////
  double A_plane[3][3];
  double norm = Vec3d_normalize(A_plane[2], dir);

  if (norm==0.0)
  {
    RLOG(1, "Failed to construct polygon Eigen frame - Eigenvector is 0");
    MatNd_destroy(tmpVerts);
    return NULL;
  }

  Vec3d_crossProduct(A_plane[0], Vec3d_ey(), A_plane[2]);
  Vec3d_normalizeSelf(A_plane[0]);
  Vec3d_crossProduct(A_plane[1], A_plane[2], A_plane[0]);

  HTr A_CI;   // Transformation from (I)nertial to (P)olygon centroid frame
  Vec3d_copy(A_CI.org, centroid);
  Mat3d_copy(A_CI.rot, A_plane);

  bool matOk = Mat3d_isValid(A_CI.rot);

  if (matOk==false)
  {
    RLOG(1, "Polygon Eigen frame is not a valid rotation matrix");
    MatNd_destroy(tmpVerts);
    return NULL;
  }

  //////////////////////////////////////////////////////////
  // A_CB: From body to contact frame
  // A_BI: From world to body
  // A_PI: From world to parent
  // A_CP = what we are looking for
  // Compute world to contact polygon transform
  //////////////////////////////////////////////////////////

  // Rotate so that the y-axis always points up vertically
  double rho = asin(A_CI.rot[0][2]);   // Vertical component of x-axis
  Mat3d_rotateSelfAboutXYZAxis(A_CI.rot, 2, -rho);

  // Transformation from world into polygon centroid frame
  HTr_copy(A_polyWorld, &A_CI);

  // To compute the Eigenbasis, we substracted the centroid from the
  // vertices. Now we have to add them again to have them correctly in
  // world coordinates.
  for (unsigned int i=0; i<tmpVerts->m; ++i)
  {
    double* row_i = MatNd_getRowPtr(tmpVerts, i);
    Vec3d_addSelf(row_i, centroid);
  }


  // Vertices are returned in world coordinates
  return tmpVerts;
}



bool PolyROSComponent::updatePolygon(const geometry_msgs::PolygonStamped::ConstPtr& rosPoly)
{
  HTr A_polyWorld;
  MatNd* tmpVerts = computePolyFrame(rosPoly, &A_polyWorld);

  if (tmpVerts==NULL)
  {
    return false;
  }

  trfMtx.lock();

  if (T_polyI==NULL)
  {
    this->T_polyI = HTr_clone(&A_polyWorld);
  }
  else
  {
    HTr_copy(this->T_polyI, &A_polyWorld);
  }

  if ((T_camI==NULL) || (T_objI==NULL))
  {
    trfMtx.unlock();
    RLOG(1, "Camera and body transforms not found");
    return false;
  }

  HTr A_camI, A_objI;
  HTr_copy(&A_camI, T_camI);
  HTr_copy(&A_objI, T_objI);
  trfMtx.unlock();

#if defined (WITH_GRAPHICS)
  //////////////////////////////////////////////////////////
  // Schedule the old geometries for deletion in the next
  // event loop update.
  //////////////////////////////////////////////////////////
  removeGraphicsNode("IK", graphicsNodeName);
  removeGraphicsNode(graphicsNodeName);

  //////////////////////////////////////////////////////////
  // Update visualization. Here we show the raw polygon
  // points in world coordinates to make sure the
  // projections work correctly.
  //////////////////////////////////////////////////////////
#if 0
  osg::ref_ptr<VertexArrayNode> polyNode;
  polyNode = new VertexArrayNode(osg::PrimitiveSet::POINTS);
  polyNode->setMaterial("YELLOW");
  polyNode->copyPoints(tmpVerts);
  polyNode->setPointSize(5);
  addGraphicsNode(polyNode);
#endif
#endif


  //////////////////////////////////////////////////////////
  // Transform vertices into centroid coordinates. Here is
  // how we create the centroidal frame:
  // z-axis points along the plane's Eigenvector that is
  //   normal to the plane and directed towards the camera
  // x-axis lies in the horizondal plane
  // y-axis is right-handed to both x any z
  //////////////////////////////////////////////////////////
  size_t nVertices = tmpVerts->m;
  for (size_t i=0; i<nVertices; ++i)
  {
    double* row_i = MatNd_getRowPtr(tmpVerts, i);
    Vec3d_invTransformSelf(row_i, &A_polyWorld);
  }

  // The z-column is zero, it is deleted since the grasp detector operates
  // on 2-d arrays.
  MatNd_deleteColumn(tmpVerts, 2);

  //////////////////////////////////////////////////////////
  // Determine grasp points (in centroid coordinates)
  //////////////////////////////////////////////////////////
  graspDetector.setPolygon(tmpVerts);
  MatNd* contacts = MatNd_create(1,1);
  MatNd* normals = MatNd_create(1,1);
  graspDetector.getContacts(contacts, normals);

  //////////////////////////////////////////////////////////
  // Create an object model and publish it so that the
  // planned gets updated and visualized.
  //////////////////////////////////////////////////////////
  PolygonObjectModel mdl;
  mdl.setPolygon(contacts, normals);
  getEntity()->publish("SetPolygon", mdl);


#if defined (WITH_GRAPHICS)
  //////////////////////////////////////////////////////////
  // Visualize. Centroid and dir are in body coordinates.
  //////////////////////////////////////////////////////////
  osg::ref_ptr<osg::Group> visGroup = new osg::Group();
  visGroup->addChild(new Rcs::COSNode(0.25));


  //////////////////////////////////////////////////////////
  // Visualize grasp points. These are represented in the
  // centroidal frame (P) and need to be transformed into
  // the world frame (I).
  //////////////////////////////////////////////////////////
  auto nodes = graspDetector.createVisualizations(0.05, 0);
  for (size_t i=0; i<nodes.size(); ++i)
  {
    visGroup->addChild(nodes[i]);

    // Mirror polygon on other side
    double z = 0.0;
    double x = 0.0;
    osg::ref_ptr<osg::MatrixTransform> mirrorMat = new osg::MatrixTransform;
    mirrorMat->preMult(osg::Matrix::translate(x, 0.0, z)*
                       osg::Matrix::scale(-1.0, 1.0, 1.0)*
                       osg::Matrix::translate(-x, 0.0, z));

    for (size_t i=0; i<nodes.size(); ++i)
    {
      mirrorMat->addChild(nodes[i]);
    }

    addGraphicsNode(mirrorMat, "IK", "Partner_Box_v");
  }

  double extrudeDir[3];
  Vec3d_set(extrudeDir, 0.0, 0.0, -0.75);
  visGroup->addChild(graspDetector.createVis3d(extrudeDir));

  addGraphicsNode(visGroup, "IK", objectBdy);
#endif

  //////////////////////////////////////////////////////////
  // Clean up.
  //////////////////////////////////////////////////////////
  MatNd_destroy(contacts);
  MatNd_destroy(normals);
  MatNd_destroy(tmpVerts);


  return true;
}

// We subscribed to ROS only when we are not in debugging mode (which means
// reading polygons through giving a file name through the ReloadPolygon
// event)
void PolyROSComponent::onStart()
{
#if defined (USE_ROS)
  RLOG_CPP(0, "PolyROSComponent::onStart():: Subscribing to "
           << rosFullTopicName);
  if (nh == nullptr)
  {
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  }

  this->polySubscriber = nh->subscribe(rosFullTopicName, 10,
                                       &PolyROSComponent::rosCallback, this);
#endif
}

// We subscribed to ROS only when we are not in debugging mode (which means
// reading polygons through giving a file name through the ReloadPolygon
// event)
void PolyROSComponent::onStop()
{
#if defined (USE_ROS)
  RLOG(0, "PolyROSComponent::onStop()");
  if (nh)
  {
    this->polySubscriber.shutdown();
  }

#endif
}

// This is called from both ROS and debug mode. In debug mode, this function
// is executed in its own thread each time the onSetNewData() method is
// called (through the event "ReloadPolygon")
void PolyROSComponent::rosCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  if (this->listenToROS==false)
  {
    return;
  }

  RLOG_CPP(1, "rosCallback");

  // We only consider an incoming polygon to bevalid if it has at
  // least 3 vertices.
  const size_t nVertices = msg->polygon.points.size();
  RLOG_CPP(1, "Received polygon with " << nVertices << " points");

  if (nVertices > 2)
  {
    trfMtx.lock();
    this->cameraBdy = msg->header.frame_id;
    trfMtx.unlock();
    RLOG(1, "Incoming message with frame_id %s", msg->header.frame_id.c_str());
  }
  // End concurrent code



  // Before updating the polygon, we need to wait until the other thread has
  // picked up the camera and object body. This must happen after assigning the
  // camera body, and before reading the camera transform.
  // \todo: Should we do a try_lock() here and return? Otherwise the
  //        subscriber queue might fill up
  initMtx.lock();
  bool success = updatePolygon(msg);
  initMtx.unlock();

  RLOG(1, "%s updating polygon", success ? "Success" : "Failure");
}

// This is only subscribed to when we are in debug mode (reading files
// instead of reading from ROS)
void PolyROSComponent::onSetNewData(std::string fileName)
{
  // Here we manually set this to the name given in the xml file. This allows
  // to attach the camera to the kinect.
  trfMtx.lock();
  this->cameraBdy = "kinect2_dexco";
  trfMtx.unlock();

  if (fileName=="Box")
  {
    PolygonObjectModel mdl;
    mdl.setBoxPolygon(0.63, 0.36);

    // Publishes box polygon
    getEntity()->publish("SetPolygon", mdl);

#if defined (WITH_GRAPHICS)
    removeGraphicsNode(graphicsNodeName);
    removeGraphicsNode("IK", graphicsNodeName);
    double center[3];
    Vec3d_set(center, 0.0, 0.0, -0.375);
    osg::ref_ptr<Rcs::BoxNode> box = new Rcs::BoxNode(center, 0.63, 0.36, 0.75);
    box->setWireframe(false);
    box->setMaterial("DARKRED_TRANS", 0.5);
    addGraphicsNode(box, "IK", objectBdy);
#endif
  }
  else if (fileName=="Cylinder")
  {
    PolygonObjectModel mdl;
    mdl.setCylinderPolygon(0.3);

    // Publishes cylinder polygon
    getEntity()->publish("SetPolygon", mdl);

#if defined (WITH_GRAPHICS)
    removeGraphicsNode(graphicsNodeName);
    removeGraphicsNode("IK", graphicsNodeName);
    double cCntr[3];
    Vec3d_set(cCntr, 0.0, 0.0, -0.375);
    osg::ref_ptr<Rcs::CylinderNode> cyl = new Rcs::CylinderNode(cCntr, NULL, 0.3, 0.75);
    cyl->setWireframe(false);
    cyl->setMaterial("DARKRED_TRANS", 0.5);
    addGraphicsNode(cyl, "IK", objectBdy);
#endif
  }
  else if (fileName=="LShape")
  {
    PolygonObjectModel mdl;
    mdl.setLShapePolygon(0.5, 0.5);

    // Publishes L-shaped polygon
    getEntity()->publish("SetPolygon", mdl);

#if defined (WITH_GRAPHICS)
    removeGraphicsNode(graphicsNodeName);
    removeGraphicsNode("IK", graphicsNodeName);
    double center[3];
    Vec3d_set(center, 0.08, -0.1075, -0.375);
    osg::ref_ptr<Rcs::BoxNode> l1 = new Rcs::BoxNode(center, 0.5, 0.125, 0.75);
    l1->setWireframe(false);
    l1->setMaterial("DARKRED_TRANS", 0.5);
    Vec3d_set(center, -0.1075, 0.1425, -0.375);
    osg::ref_ptr<Rcs::BoxNode> l2 = new Rcs::BoxNode(center, 0.125, 0.375, 0.75);
    l2->setWireframe(false);
    l2->setMaterial("DARKRED_TRANS", 0.5);
    osg::ref_ptr<osg::Group> lshape = new osg::Group();
    lshape->addChild(l1.get());
    lshape->addChild(l2.get());
    addGraphicsNode(lshape, "IK", objectBdy);
#endif
  }
  else
  {
    RLOG(1, "PolyROSComponent::onSetNewData(%s)", fileName.c_str());
    auto tmpPoly = polyFromFile(fileName.c_str());

    // Call as thread function. This makes sure we don't get blocked waiting
    // for the camera transform which gets updated by the event loop (and
    // set by the rosCallback)
    std::thread t1(&PolyROSComponent::rosCallback, this, tmpPoly);
    t1.detach();
  }
}

geometry_msgs::PolygonStamped::ConstPtr PolyROSComponent::polyFromFile(std::string fileName) const
{
#if !defined (USE_ROS)
  auto tmpPoly = std::make_shared<geometry_msgs::PolygonStamped>();
#else
  auto tmpPoly = boost::make_shared<geometry_msgs::PolygonStamped>();
#endif
  tmpPoly->header.frame_id = "kinect2_dexco";
  MatNd* poly = MatNd_createFromFile(fileName.c_str());

  if (poly==NULL)
  {
    RLOG(1, "Couldn't open polygon file %s - returning empty polygon",
         fileName.c_str());
    return tmpPoly;
  }

  if (poly->n==2)
  {
    RLOG(1, "Found polygon with 2 columns - appending zero-column");
    MatNd* col3 = MatNd_create(poly->m, 1);
    MatNd_appendColumns(poly, col3);
    MatNd_destroy(col3);
  }
  RCHECK(poly->m > 2);

  for (unsigned int i=0; i<poly->m; ++i)
  {
    const double* row_i = MatNd_getRowPtr(poly, i);
    geometry_msgs::Point32 pt;
    pt.x = row_i[0];
    pt.y = row_i[1];
    pt.z = row_i[2];
    tmpPoly->polygon.points.push_back(pt);
  }
  MatNd_destroy(poly);

  return tmpPoly;
}

void PolyROSComponent::onListenToROS(bool enable)
{
  RLOG(0, "%s ROS subscriber", enable ? "Subscribing" : "Unsubscribing");
  this->listenToROS = enable;
}

// We assign names here at the last point before publishing. This allows to
// consistently delete all published nodes with the same name.
void PolyROSComponent::addGraphicsNode(osg::ref_ptr<osg::Node> node,
                                       std::string graphID,
                                       std::string parent) const
{
  node->setName(graphicsNodeName);
  ntt->publish<osg::ref_ptr<osg::Node>,
      std::string,
      std::string>("AddChildNode", node, graphID, parent);
}

void PolyROSComponent::addGraphicsNode(osg::ref_ptr<osg::Node> node) const
{
  node->setName(graphicsNodeName);
  ntt->publish<osg::ref_ptr<osg::Node>>("AddNode", node);
}

void PolyROSComponent::removeGraphicsNode(std::string graphID,
                                          std::string nodeName) const
{
  ntt->publish<std::string, std::string>("RemoveNode", graphID, nodeName);
}

void PolyROSComponent::removeGraphicsNode(std::string nodeName) const
{
  ntt->publish<std::string, std::string>("RemoveNode", "", nodeName);
}

}   // namespace
