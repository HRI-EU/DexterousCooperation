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

#include "GraphicsWindow.h"
#include "EntityBase.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_Vec3d.h>

#include <thread>
#include <iostream>
#include <algorithm>




namespace Rcs
{

class MapItem : public GraphNode
{
public:

  virtual ~MapItem()
  {
    RcsGraph_destroy(this->graph);
    MatNd_destroy(this->q_buf);
    MatNd_destroy(this->q_dot_buf);

    for (size_t i=0; i<sensorData.size(); ++i)
    {
      MatNd_destroy(sensorData[i]);
    }

    sensorData.clear();
  }

  bool realized() const
  {
    if (realizeMtx.try_lock() == true)
    {
      this->realizeMtx.unlock();
      return true;
    }

    return false;
  }

  void copyState(const RcsGraph* other, bool resizeable)
  {
    if (realized())
    {
      stateCopyingMtx.lock();
      MatNd_copy(this->q_buf, other->q);
      MatNd_copy(this->q_dot_buf, other->q_dot);

      size_t sIdx = 0;
      RCSGRAPH_FOREACH_SENSOR(other)
      {
        MatNd_copy(sensorData[sIdx], SENSOR->rawData);
        sIdx++;
      }

      if (resizeable)
      {
        RcsGraph_copy(graph, other);

        // RcsBody* bSrc = other->root;
        // RcsBody* bDst = graph->root;

        // while (bSrc && bDst)
        // {
        //   bSrc = RcsBody_depthFirstTraversalGetNext(bSrc);
        //   bDst = RcsBody_depthFirstTraversalGetNext(bDst);

        //   if (bSrc && bDst)
        //   {
        //     RcsShape** sSrc = bSrc->shape;
        //     RcsShape** sDst = bDst->shape;

        //     while ((*sSrc) && (*sDst))
        //     {
        //       Vec3d_copy((*sDst)->extents, (*sSrc)->extents);
        //       sSrc++;
        //       sDst++;
        //     }
        //   }
        // }
      }


      stateCopyingMtx.unlock();
    }
  }

  void pasteState()
  {
    if (realized())
    {
      stateCopyingMtx.lock();
      MatNd_copy(graph->q, this->q_buf);
      MatNd_copy(graph->q_dot, this->q_dot_buf);

      size_t sIdx = 0;
      RCSGRAPH_FOREACH_SENSOR(this->graph)
      {
        MatNd_copy(SENSOR->rawData, sensorData[sIdx]);
        sIdx++;
      }
      stateCopyingMtx.unlock();
    }
  }

  static void realizeNodeInThread(GraphicsWindow* window, std::string eventName,
                                  const RcsGraph* other, bool resizeable)
  {
    std::thread t1(&MapItem::constructNode, window, eventName, other, resizeable);
    t1.detach();
  }

  static osg::ref_ptr<MapItem> getEntry(std::string eventName)
  {
    osg::ref_ptr<MapItem> mi;

    mapMtx.lock();
    auto nd = MapItem::eventMap.find(eventName);

    if (nd != eventMap.end())
    {
      mi = nd->second;
    }
    mapMtx.unlock();

    return mi;
  }

  static osg::ref_ptr<MapItem> eraseEntry(std::string eventName)
  {
    osg::ref_ptr<MapItem> mi;

    mapMtx.lock();
    auto nd = MapItem::eventMap.find(eventName);

    if (nd != eventMap.end())
    {
      mi = nd->second;
      MapItem::eventMap.erase(nd);
    }
    mapMtx.unlock();

    return mi;
  }



  // This function is atomic in a way that the event will added to the map in a
  // thread-safe way before returning.
  static osg::ref_ptr<MapItem> getOrCreateEntry(std::string eventName, bool& exists)
  {
    mapMtx.lock();
    auto nd = MapItem::eventMap.find(eventName);
    osg::ref_ptr<MapItem> mi;

    if (nd == eventMap.end())
    {
      exists = false;
      mi = new MapItem();
      MapItem::eventMap[eventName] = mi;
    }
    else
    {
      exists = true;
      mi = nd->second;
    }
    mapMtx.unlock();

    return mi;
  }

  static void clear(Viewer* viewer)
  {
    mapMtx.lock();
    std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::eventMap;
    eventMap.clear();
    mapMtx.unlock();

    for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
    {
      osg::ref_ptr<MapItem> mi = it->second;

      if (mi->realized())
      {
        viewer->removeNode(mi);
      }
    }

  }

  static void clearAllNodesInThread(GraphicsWindow* window)
  {
    std::thread t1(&MapItem::clear, window);
    t1.detach();
  }

  static void print()
  {
    int count = 0;

    mapMtx.lock();
    if (eventMap.empty())
    {
      RLOG(5, "No RenderGraph events");
    }
    else
    {
      for (auto it = eventMap.begin(); it != eventMap.end(); ++it)
      {
        RLOG(5, "RenderGraph(%d) = %s", count++, it->first.c_str());
      }
    }
    mapMtx.unlock();
  }

  static std::map<std::string,osg::ref_ptr<MapItem>> getEventMap()
  {
    mapMtx.lock();
    std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::eventMap;
    mapMtx.unlock();

    return cpyOfMap;
  }

  static void setBodyActivation(std::string bodyName, bool enable)
  {
    RLOG(1, "Setting body activation of %s to %s",
         bodyName.c_str(), enable ? "TRUE" : "FALSE");


    mapMtx.lock();

    auto it = std::find(deactivatedBodies.begin(), deactivatedBodies.end(),
                        bodyName);

    if (enable == false)   // Disable
    {
      if (it == deactivatedBodies.end())
      {
        deactivatedBodies.push_back(bodyName);
      }
    }
    else   // Enable
    {
      if (it != deactivatedBodies.end())
      {
        deactivatedBodies.erase(it);
      }
    }

    mapMtx.unlock();
  }

  RcsGraph* getGraph() const
  {
    return this->graph;
  }

private:

  MapItem() : GraphNode(), graph(NULL), q_buf(NULL), q_dot_buf(NULL)
  {
    this->realizeMtx.lock();
  }

  static void constructNode(GraphicsWindow* window, std::string eventName,
                            const RcsGraph* other, bool resizeable)
  {
    RLOG(5, "Constructing node for event \"%s\"", eventName.c_str());

    osg::ref_ptr<MapItem> mi = getEntry(eventName);
    RCHECK_MSG(mi, "eventName %s not found", eventName.c_str());
    mi->graph = RcsGraph_clone(other);

    bool success = mi->init(mi->graph, resizeable, false);

    if (success==false)
    {
      RLOG(4, "Failed to initialize GraphNode for %s", eventName.c_str());
    }

    window->add(mi);

    mi->q_buf = MatNd_clone(mi->graph->q);
    mi->q_dot_buf = MatNd_clone(mi->graph->q_dot);

    RCSGRAPH_FOREACH_SENSOR(mi->graph)
    {
      RcsSensor* otherSensor = RcsGraph_getSensorByName(other, SENSOR->name);
      if (otherSensor != NULL)
      {
        mi->sensorData.push_back(MatNd_clone(otherSensor->rawData));
      }
    }

    mapMtx.lock();
    for (size_t i = 0; i < deactivatedBodies.size(); ++i)
    {
      BodyNode* bnd = mi->getBodyNode(deactivatedBodies[i].c_str());

      if (bnd != NULL)
      {
        RLOG(5, "Hiding deactivated body %s", deactivatedBodies[i].c_str());
        bnd->setVisibility(false);
      }
    }
    mapMtx.unlock();

    mi->realizeMtx.unlock();
  }

  MapItem(const MapItem&);
  MapItem& operator=(const MapItem&);

  mutable std::mutex realizeMtx;
  mutable std::mutex stateCopyingMtx;
  RcsGraph* graph;
  MatNd* q_buf;
  MatNd* q_dot_buf;
  std::vector<MatNd*> sensorData;

  static std::map<std::string,osg::ref_ptr<MapItem>> eventMap;
  static std::vector<std::string> deactivatedBodies;
  static std::mutex mapMtx;
};


std::map<std::string,osg::ref_ptr<MapItem>> MapItem::eventMap;
std::mutex MapItem::mapMtx;
std::vector<std::string> MapItem::deactivatedBodies;

} // namespace

























namespace Rcs
{

GraphicsWindow::GraphicsWindow(EntityBase* parent, bool startWithStartEvent,
                               bool synWithEventLoop_, bool simpleGraphics) :
  Rcs::ComponentBase(parent),
  Viewer(!simpleGraphics, !simpleGraphics),
  synWithEventLoop(synWithEventLoop_),
  resizeable(false)
{
  pthread_mutex_init(&frameMtx, NULL);

#if defined(_MSC_VER)
  setWindowSize(12, 36, 640, 480);
#else
  setWindowSize(0, 0, 1280, 960);
  //setWindowSize(0, 0, 640, 480);
#endif

  setCameraTransform(4.105175, 3.439, 2.574,   0.233, -0.286, -2.28);

  this->keyCatcher = new Rcs::KeyCatcher();
  add(keyCatcher.get());
  this->hud = new Rcs::HUD();
  add(hud.get());
  this->vertexNode = new Rcs::VertexArrayNode();
  //vertexNode->hide();
  add(vertexNode.get());

  subscribeAll();

  if (startWithStartEvent==false)
  {
    start();
  }
}

void GraphicsWindow::subscribeAll()
{
  subscribe("Stop", &GraphicsWindow::stop);
  subscribe("RenderGraph", &GraphicsWindow::onRender);
  subscribe("RenderLines", &GraphicsWindow::onRenderLines);
  subscribe("RenderCommand", &GraphicsWindow::onRenderCommand);
  subscribe("RenderClear", &GraphicsWindow::clear);
  subscribe("Print", &GraphicsWindow::print);
  subscribe("SetText", &GraphicsWindow::setText);
  subscribe("SetTextLine", &GraphicsWindow::setTextLine);
  subscribe("AddText", &GraphicsWindow::addText);
  subscribe("ClearText", &GraphicsWindow::clearText);
  subscribe("SetObjectActivation", &GraphicsWindow::onObjectActivation);
  subscribe("EmergencyStop", &GraphicsWindow::onEmergencyStop);
  subscribe("EmergencyRecover", &GraphicsWindow::onEmergencyRecover);
  subscribe("Start", &GraphicsWindow::start);
  subscribe("AddNode", &GraphicsWindow::onAddNode);
  subscribe("AddChildNode", &GraphicsWindow::onAddChildNode);
  subscribe("RemoveNode", &GraphicsWindow::onRemoveNode);
  subscribe("SetObjectColor", &GraphicsWindow::onObjectColor);

  if (this->synWithEventLoop)
  {
    subscribe("Render", &GraphicsWindow::frame);
  }

}

GraphicsWindow::~GraphicsWindow()
{
  stop();
  MapItem::clear(this);
  pthread_mutex_destroy(&frameMtx);
}

void GraphicsWindow::start()
{
  // If no viewer has been launched before the start event is called, we call
  // the Viewer's create() function to initialize the graphics window etc.
  if (!viewer.valid())
  {
    create(true, true);
  }

  // Only in case the viewer runs in its own thread, we launch up the thread
  // function here.
  if (!this->synWithEventLoop)
  {
    runInThread(&frameMtx);
  }
}

void GraphicsWindow::stop()
{
  RLOG(5, "Stop::stop()");
  stopUpdateThread();   // Blocks until thread has been joined
}

void GraphicsWindow::clear()
{
  MapItem::clearAllNodesInThread(this);
}

const RcsGraph* GraphicsWindow::getGraphById(std::string graphId)
{
  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi && mi->realized())
  {
    return mi->getGraphPtr();
  }

  return NULL;
}

GraphNode* GraphicsWindow::getGraphNodeById(std::string graphId)
{
  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi == NULL)
  {
    RLOG(1, "GraphNode \"%s\" not found", graphId.c_str());
    return NULL;
  }

  if (!mi->realized())
  {
    RLOG(1, "GraphNode \"%s\" not realized", graphId.c_str());
    return NULL;
  }

  return mi;
}

std::string GraphicsWindow::getName() const
{
  return std::string("GraphicsWindow");
}

void GraphicsWindow::onAddNode(osg::ref_ptr<osg::Node> node)
{
  if (!node.valid())
  {
    RLOG(1, "Can't add invalid osg::Node - skipping");
    return;
  }

  if (!isRealized())
  {
    RLOG_CPP(1, "Couldn't add node " << node->getName()
             << " before viewer is realized - republishing node");
    getEntity()->publish("AddNode", node);
    return;
  }

  add(node.get());
}

void GraphicsWindow::onAddChildNode(osg::ref_ptr<osg::Node> node,
                                    std::string graphId,
                                    std::string parent)
{
  if (!node.valid())
  {
    RLOG(1, "Can't add invalid osg::Node - skipping");
    return;
  }

  RLOG(5, "GraphicsWindow::onAddChildNode(%s, %s, %s)",
       node->getName().c_str(), graphId.c_str(), parent.c_str());

  if (!isRealized())
  {
    RLOG_CPP(1, "Couldn't add child node " << node->getName()
             << " before viewer is realized - republishing node");
    getEntity()->publish("AddChildNode", node, graphId, parent);
    return;
  }

  GraphNode* gnd = getGraphNodeById(graphId);
  if (!gnd)
  {
    // \todo: Allow to append to non-graph nodes
    RLOG(5, "Can't find GraphNode with id %s - skipping", graphId.c_str());
    return;
  }

  BodyNode* bnd = gnd->getBodyNode(parent.c_str());
  if (!bnd)
  {
    RLOG(5, "Can't find parent node with name %s in graph %s- skipping",
         graphId.c_str(), parent.c_str());
    return;
  }

  RLOG(5, "GraphicsWindow: adding child to %s", bnd->body()->name);
  add(bnd, node.get());
}

void GraphicsWindow::onRemoveNode(std::string graphId, std::string nodeName)
{
  RLOG(5, "GraphicsWindow::onRemoveNode(%s, %s)",
       graphId.c_str(), nodeName.c_str());

  // If no graphId is given, we search through the viewer's rootNode
  if (graphId.empty())
  {
    removeNode(nodeName);
  }


  GraphNode* gnd = getGraphNodeById(graphId);
  if (!gnd)
  {
    RLOG(5, "Can't find GraphNode with id \"%s\" - skipping", graphId.c_str());
    return;
  }

  // Search through the GraphNode. We do this in a while loop to remove all
  // nodes with the same name
  removeNode(gnd, nodeName);
}

void GraphicsWindow::onRenderCommand(std::string graphId, std::string command)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept render command before graphics window is launched"
         " (graph id \"%s\", command \"%s\"", graphId.c_str(), command.c_str());
    getEntity()->publish<std::string,std::string>("RenderCommand",
                                                  graphId, command);
    return;
  }

  if (STRCASEEQ("ShowLines", graphId.c_str()))
  {
    if (command=="true")
    {
      vertexNode->show();
    }
    else
    {
      vertexNode->hide();
    }
  }

  if (STRCASEEQ("BackgroundColor", graphId.c_str()))
  {
    if (command.empty())
    {
      setBackgroundColor(getDefaultBackgroundColor());
    }
    else
    {
      setBackgroundColor(command);
    }
  }

  if (STRCASEEQ("CameraTransform", graphId.c_str()))
  {
    HTr A;
    bool success = HTr_fromString(&A, command.c_str());
    if (success)
    {
      setCameraTransform(&A);
    }
    else
    {
      RLOG(1, "Failed to get camera transform from \"%s\"", command.c_str());
    }
  }

  if (STRCASEEQ("ResetView", graphId.c_str()))
  {
    resetView();
  }

  if (STRCASEEQ("FieldOfView", graphId.c_str()))
  {
    int nStrings = String_countSubStrings(command.c_str(), " ");

    if (nStrings==1)
    {
      double fov = String_toDouble_l(command.c_str());
      setFieldOfView(RCS_DEG2RAD(fov));
      RLOG(5, "FOV = %f rad (%f degrees)", RCS_DEG2RAD(fov), fov);
    }
    else if (nStrings==2)
    {
      double fov[2];
      String_toDoubleArray_l(command.c_str(), fov, 2);
      setFieldOfView(RCS_DEG2RAD(fov[0]), RCS_DEG2RAD(fov[1]));
      RLOG(5, "FOV = %f %f degrees", fov[0], fov[1]);
    }
  }

  if (STRCASEEQ("hud", graphId.c_str()))
  {
    if (STRCASEEQ("hide", command.c_str()))
    {
      hud->hide();
    }
    else if (STRCASEEQ("show", command.c_str()))
    {
      hud->show();
    }
    else if (STRCASEEQ("toggle", command.c_str()))
    {
      hud->toggle();
    }
  }

  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi == NULL)
  {
    RLOG(6, "Couldn't find graph for id \"%s\" - skipping command \"%s\"",
         graphId.c_str(), command.c_str());
    return;
  }

  if (!mi->realized())
  {
    RLOG(5, "GraphNode for id \"%s\" not realized - queuing command \"%s\"",
         graphId.c_str(), command.c_str());
    getEntity()->publish<std::string,std::string>("RenderCommand", graphId,
                                                  command);
    return;
  }

  GraphNode* gn = mi;

  if (command=="setGhostMode")
  {
    RLOG(5, "Setting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(true, "RED");
    unlock();
  }
  else if (command=="unsetGhostMode")
  {
    RLOG(5, "Unsetting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(false);
    unlock();
  }
  else if (command=="toggleGraphicsModel")
  {
    lock();
    gn->toggleGraphicsModel();
    unlock();
  }
  else if (command=="togglePhysicsModel")
  {
    lock();
    gn->togglePhysicsModel();
    unlock();
  }
  else if (command=="toggleCollisionModel")
  {
    lock();
    gn->toggleCollisionModel();
    unlock();
  }
  else if (command=="toggleReferenceFrames")
  {
    lock();
    gn->toggleReferenceFrames();
    unlock();
  }
  else if (command=="show")
  {
    lock();
    gn->show();
    unlock();
  }
  else if (command=="hide")
  {
    lock();
    gn->hide();
    unlock();
  }
  else if (command=="erase")
  {
    osg::ref_ptr<MapItem> mi = MapItem::eraseEntry(graphId);
    removeNode(mi);
  }
  else if (command=="model_state")
  {
    RcsGraph_fprintModelState(stdout, gn->getGraphPtr(),
                              gn->getGraphPtr()->q, NULL, 0);
  }

}

void GraphicsWindow::onRender(std::string graphId, const RcsGraph* other)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept graph update before graphics window is launched "
         "(graph id \"%s\"", graphId.c_str());
    getEntity()->publish<std::string,const RcsGraph*>("RenderGraph",
                                                      graphId, other);
    return;
  }

  bool exists;
  osg::ref_ptr<MapItem> mi = MapItem::getOrCreateEntry(graphId, exists);

  if (exists == false)
  {
    RLOG(5, "Creating %s GraphNode \"%s\"",
         getResizeable() ? "resizeable" : "non-resizeable", graphId.c_str());
    MapItem::realizeNodeInThread(this, graphId, other, getResizeable());
  }

  // Copies the joint and sensor values for all realized GraphNodes. This is
  // mutex-protected against the pasteState() method in the frame() function.
  mi->copyState(other, getResizeable());
}

void GraphicsWindow::onRenderLines(const MatNd* array)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept array update before graphics window is launched");
    getEntity()->publish<const MatNd*>("RenderLines", array);
    return;
  }

  vertexNode->copyPoints(array);
}

void GraphicsWindow::frame()
{
  if (isInitialized() == false)
  {
    init();
  }

  // Mutex only around map copying so that we don't need to wait until
  // forward kinematics is computed.
  auto cpyOfMap = MapItem::getEventMap();

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); it++)
  {
    auto mi = it->second;
    mi->pasteState();
  }

  lock();

  // Mutex only around map copying so that we don't need to wait until
  // forward kinematics is computed.
  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); it++)
  {
    auto mi = it->second;

    if (mi->realized())
    {
      RcsGraph_setState(mi->getGraph(), NULL, NULL);
    }

  }
  unlock();

  Viewer::frame();
  handleKeys();
}

void GraphicsWindow::handleKeys()
{
  if (!keyCatcher.valid())
  {
    return;
  }

  if (keyCatcher->getAndResetKey('q'))
  {
    getEntity()->publish<>("Quit");
  }
  else if (keyCatcher->getAndResetKey('x'))
  {
    static int viewMode = 0;
    viewMode++;
    if (viewMode>2)
    {
      viewMode = 0;
    }

    switch (viewMode)
    {
      case 0:
        RLOG(0, "Showing physics and IK model");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("setGhostMode"));
        break;

      case 1:
        RLOG(0, "Showing IK model only");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("hide"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("unsetGhostMode"));
        break;


      case 2:
        RLOG(0, "Showing physics model only");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("hide"));
        break;
    }
  }

  for (auto const& entry : keyCallbacks)
  {
    if (keyCatcher->getAndResetKey(entry.first))
    {
      entry.second(entry.first);
    }
  }
}

void GraphicsWindow::setText(std::string text)
{
  std::vector<std::string> lines;
  size_t pos = text.find_first_of("\n", 0);
  size_t lastPos = 0;
  while (pos != std::string::npos)
  {
    std::string line = text.substr(lastPos, pos - lastPos);
    lines.push_back(line);
    lastPos = pos + 1;
    pos = text.find_first_of("\n", lastPos);
  }
  lines.push_back(text.substr(lastPos, pos - lastPos));
  if (lines.size() > hudText.size())
  {
    hudText.resize(lines.size());
  }

  for (size_t i = 0; i < lines.size(); i++)
  {
    hudText[i] = lines[i];
  }

  updateText();
}

void GraphicsWindow::setTextLine(std::string text, int lineNum)
{
  if (lineNum < 0)
  {
    return;
  }

  if (text == "")
  {
    if (lineNum == (int) hudText.size() - 1)
    {
      hudText.pop_back();
    }
    else if (lineNum < (int) hudText.size())
    {
      hudText[lineNum] = text;
    }
  }
  else
  {
    if (lineNum >= (int) hudText.size())
    {
      hudText.resize(lineNum + 1);
    }
    hudText[lineNum] = text;
  }

  updateText();
}

void GraphicsWindow::updateText()
{
  std::string textNew = "";
  for (size_t i = 0; i < hudText.size(); i++)
  {
    textNew += hudText[i];
    if (i != hudText.size() - 1)
    {
      textNew += "\n";
    }
  }

  hud->setText(textNew);
}

void GraphicsWindow::addText(std::string text)
{
  hudText.push_back(text);
  updateText();
}

void GraphicsWindow::clearText()
{
  hudText.clear();
  hud->clearText();
  hud->resize(0,0);
}

void GraphicsWindow::setKeyCallback(char key, std::function<void(char)> cb,
                                    std::string description, std::string group)
{
  if (keyCallbacks.find(key) != keyCallbacks.end())
  {
    RWARNING(0, "WARNING: Over-writing previously assigned key: '%c'.", key);
    keyCatcher->deregisterKey(std::string(1, key), group);
  }

  keyCatcher->registerKey(std::string(1, key), description, group);
  keyCallbacks[key] = cb;
}

void GraphicsWindow::clearKeyCallback(char key)
{
  if (keyCallbacks.find(key) != keyCallbacks.end())
  {
    keyCatcher->deregisterKey(std::string(1, key));
    keyCallbacks.erase(key);
  }
}

void GraphicsWindow::print()
{
  MapItem::print();
}

pthread_mutex_t* GraphicsWindow::getFrameMtx() const
{
  return &this->frameMtx;
}

void GraphicsWindow::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  getEntity()->publish("SetTextLine", std::string("EmergencyStop"), 5);
}

void GraphicsWindow::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  getEntity()->publish("SetTextLine", std::string(""), 5);
  getEntity()->publish("ClearText");
}

void GraphicsWindow::onObjectActivation(std::string objectName, bool activation)
{
  RLOG(1, "GraphicsWindow::onObjectActivation(%s, %s)", objectName.c_str(),
       activation ? "true" : "false");

  MapItem::setBodyActivation(objectName, activation);

  // if (!isRealized())
  // {
  //  RLOG(1, "Couldn't %s object %s before graphics window is launched",
  //       activation ? "activate" : "deactivate", objectName.c_str());
  //  getEntity()->publish<std::string, bool>("SetObjectActivation",
  //                                          objectName, activation);
  //  return;
  // }


  std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::getEventMap();

  RLOG_CPP(1, "Going through " << cpyOfMap.size() << " graphs");

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
  {
    std::string graphId = it->first;
    osg::ref_ptr<MapItem> mi = it->second;

    if (!mi->realized())
    {
      RLOG(1, "GraphNode \"%s\" not realized - skipping",
           mi->getGraphPtr()->cfgFile);
      // getEntity()->publish<std::string, bool>("SetObjectActivation",
      //                                         objectName, activation);
      continue;
    }

    BodyNode* bnd = mi->getBodyNode(objectName.c_str());

    if (bnd == NULL)
    {
      RLOG(1, "BodyNode \"%s\" in graph \"%s\" doesn't exist - not hiding",
           objectName.c_str(), mi->getGraphPtr()->cfgFile);
      continue;
    }

    RLOG(1, "Hiding BodyNode \"%s\" in graph \"%s\"",
         objectName.c_str(), mi->getGraphPtr()->cfgFile);

    lock();
    bnd->setVisibility(activation);
    unlock();
  }

}

void GraphicsWindow::onObjectColor(std::string whichGraph, std::string objectName, std::string color)
{
  std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::getEventMap();

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
  {
    std::string graphId = it->first;

    if ((!whichGraph.empty()) && (whichGraph!=graphId))
    {
      continue;
    }

    osg::ref_ptr<MapItem> mi = it->second;

    if (!mi->realized())
    {
      RLOG(1, "GraphNode \"%s\" not realized - trying again", mi->getGraphPtr()->cfgFile);
      getEntity()->publish<std::string,std::string,std::string>("SetObjectColor",
                                                                whichGraph, objectName, color);
      continue;
    }

    BodyNode* bnd = mi->getBodyNode(objectName.c_str());

    if (bnd == NULL)
    {
      RLOG(1, "BodyNode \"%s\" in graph \"%s\" doesn't exist - not coloring",
           objectName.c_str(), mi->getGraphPtr()->cfgFile);
      continue;
    }

    RLOG(5, "Coloring BodyNode \"%s\" in graph \"%s\"",
         objectName.c_str(), mi->getGraphPtr()->cfgFile);

    lock();
    setNodeMaterial(color, bnd);
    unlock();
  }

}

void GraphicsWindow::setResizeable(bool enable)
{
  this->resizeable = enable;
}

bool GraphicsWindow::getResizeable() const
{
  return this->resizeable;
}

}   // namespace Rcs
