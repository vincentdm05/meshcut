#include "MeshCut.hh"
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#define EDGE_CUT_POPUP "<B>Edge Cut</B><br>Cut along selected edge"
#define SELECT_EDGES_PICKMODE "MeshCut: Edge select"
#define DRAW_CUT_PICKMODE "MeshCut: Draw cut"

Q_EXPORT_PLUGIN2( meshCut , MeshCut );

MeshCut::MeshCut() :
   toolBar_(0), edgeCutAction_(0), toolBox_(0), selectButton_(0), drawButton_(0), selectionButtonToggled_(0),
   active_face_(-1), active_edge_(-1), active_vertex_(-1), active_hit_point_(0.0),
   latest_object_(0), latest_edge_(-1),
   prev_face_(-1), prev_edge_(-1), prev_hit_point_(0.0), active_edge_crossing_(0.0), prev_edge_crossing_(0.0),
   active_vertex_at_split_(-1), prev_vertex_at_split_(-1) {}

/** \brief Initialize plugin
 *
 */
void MeshCut::initializePlugin()
{
   // Cutting toolbox
   toolBox_ = new QWidget();
   QVBoxLayout* layout = new QVBoxLayout(toolBox_);

   QHBoxLayout* toggleLayout = new QHBoxLayout(toolBox_);
   toggleLayout->setSpacing(5);

   selectButton_ = new QPushButton("&Select Edges", toolBox_);
   selectButton_->setToolTip("Select edges to be cut");
   selectButton_->setCheckable(true);

   drawButton_ = new QPushButton("&Draw Cut", toolBox_);
   drawButton_->setToolTip("Draw a path to be cut");
   drawButton_->setCheckable(true);

   toggleLayout->addWidget(selectButton_);
   toggleLayout->addWidget(drawButton_);

   QPushButton* cutButton = new QPushButton("&Cut", toolBox_);
   cutButton->setToolTip("Cut selected edges.");

   layout->addItem(toggleLayout);
   layout->addWidget(cutButton);

   connect(selectButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(drawButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(cutButton, SIGNAL(clicked()), this, SLOT(slotCutSelectedEdges()));

   QIcon* toolboxIcon = new QIcon(OpenFlipper::Options::iconDirStr()+OpenFlipper::Options::dirSeparator()+"meshCut.png");
   emit addToolbox(tr("MeshCut"), toolBox_, toolboxIcon);
}

/** \brief Initialize plugin after it is loaded
 *
 */
void MeshCut::pluginsInitialized()
{
   QString iconPath = OpenFlipper::Options::iconDirStr()+OpenFlipper::Options::dirSeparator();

   // Cutting toolbar
   emit addHiddenPickMode(EDGE_CUT_POPUP);

   toolBar_ = new QToolBar("Cutting");

   QActionGroup* group = new QActionGroup(0);

   edgeCutAction_ = toolBar_->addAction( QIcon(iconPath+"meshCut.png"), EDGE_CUT_POPUP );
   edgeCutAction_->setCheckable(true);
   edgeCutAction_->setActionGroup(group);

   group->setExclusive(true);

   connect(toolBar_, SIGNAL(actionTriggered(QAction*)), this, SLOT(toolBarTriggered(QAction*)));

   emit addToolbar(toolBar_);

   // Cutting toolbox
   emit addHiddenPickMode(SELECT_EDGES_PICKMODE);
   emit addHiddenPickMode(DRAW_CUT_PICKMODE);
}

/** \brief Toolbar action trigger
 * @param _action the triggered action
 */
void MeshCut::toolBarTriggered(QAction* _action) {
   if (_action->text() == EDGE_CUT_POPUP)
      PluginFunctions::pickMode(EDGE_CUT_POPUP);

   PluginFunctions::actionMode(Viewer::PickingMode);
}

/** \brief Selection button clicked
 *
 */
void MeshCut::slotSelectionButtonClicked() {
   // Double button toggle
   if (selectButton_->isChecked() && drawButton_->isChecked()) {
      PluginFunctions::actionMode(Viewer::PickingMode);
      if (selectionButtonToggled_ == 2) {
         PluginFunctions::pickMode(SELECT_EDGES_PICKMODE);
         selectionButtonToggled_ = 1;
      } else {
         PluginFunctions::pickMode(DRAW_CUT_PICKMODE);
         selectionButtonToggled_ = 2;
      }
   } else if (selectButton_->isChecked()) {
      PluginFunctions::actionMode(Viewer::PickingMode);
      PluginFunctions::pickMode(SELECT_EDGES_PICKMODE);
      selectionButtonToggled_ = 1;
   } else if (drawButton_->isChecked()) {
      PluginFunctions::actionMode(Viewer::PickingMode);
      PluginFunctions::pickMode(DRAW_CUT_PICKMODE);
      selectionButtonToggled_ = 2;
   } else {
      PluginFunctions::actionMode(Viewer::ExamineMode);
      selectionButtonToggled_ = 0;
   }
}

/** \brief Toggle actions when the PickMode changes
 * @param _mode the new PickMode
 */
void MeshCut::slotPickModeChanged(const std::string& _mode) {
   edgeCutAction_->setChecked(_mode == EDGE_CUT_POPUP);
   selectButton_->setChecked(_mode == SELECT_EDGES_PICKMODE);
   drawButton_->setChecked(_mode == DRAW_CUT_PICKMODE);
}

/** \brief Called when the mouse is clicked
 * Only react to left mouse clicks
 *
 * @param _event The mouse click event
 */
void MeshCut::slotMouseEvent(QMouseEvent* _event) {
   if (_event->buttons() == Qt::RightButton)
      return;

   if (PluginFunctions::pickMode() == EDGE_CUT_POPUP) {
      singleEdgeCut(_event);
   } else if (PluginFunctions::pickMode() == SELECT_EDGES_PICKMODE) {
      selectEdge(_event);
   } else if (PluginFunctions::pickMode() == DRAW_CUT_PICKMODE) {
      drawLine(_event);
   }
}

/** \brief Sets closest elements as active
 * There are four elements that are set as active here, namely the index of the
 * closest face, edge and vertex, and the hit point.
 * The object container is returned so that we can access the mesh.
 *
 * @param _event the mouse click event
 */
BaseObjectData *MeshCut::setActiveElements(QMouseEvent *_event) {
   unsigned int node_idx, target_idx;
   ACG::Vec3d hit_point;
   if (PluginFunctions::scenegraphPick(ACG::SceneGraph::PICK_FACE, _event->pos(), node_idx, target_idx, &hit_point)) {
      active_hit_point_ = ACG::Vec3d(hit_point);

      BaseObjectData* object;
      if (PluginFunctions::getPickedObject(node_idx, object)) {
         // In the case of a triangle mesh
         if (object->picked(node_idx) && object->dataType(DATA_TRIANGLE_MESH)) {
            TriMesh& mesh = *PluginFunctions::triMesh(object);
            TriMesh::FaceHandle faceh = mesh.face_handle(target_idx);
            active_face_ = target_idx;

            // Get all three halfedges on the clicked face
            TriMesh::FaceEdgeIter fedge_it(mesh, faceh);
            TriMesh::HalfedgeHandle he1 = mesh.halfedge_handle(*fedge_it, 0);
            ++fedge_it;
            TriMesh::HalfedgeHandle he2 = mesh.halfedge_handle(*fedge_it, 0);
            ++fedge_it;
            TriMesh::HalfedgeHandle he3 = mesh.halfedge_handle(*fedge_it, 0);

            /// Edges
            // Find closest halfedge
            double min_e_dist = ACG::Geometry::distPointLineSquared(
                     hit_point,
                     mesh.point(mesh.to_vertex_handle(he1)),
                     mesh.point(mesh.from_vertex_handle(he1)));

            TriMesh::EdgeHandle edge = mesh.edge_handle(he1);
            double dist = ACG::Geometry::distPointLineSquared(
                     hit_point,
                     mesh.point(mesh.to_vertex_handle(he2)),
                     mesh.point(mesh.from_vertex_handle(he2)));

            if (dist < min_e_dist) {
               min_e_dist = dist;
               edge = mesh.edge_handle(he2);
            }
            dist = ACG::Geometry::distPointLineSquared(
                     hit_point,
                     mesh.point(mesh.to_vertex_handle(he3)),
                     mesh.point(mesh.from_vertex_handle(he3)));

            if (dist < min_e_dist) {
               min_e_dist = dist;
               edge = mesh.edge_handle(he3);
            }
            active_edge_ = edge.idx();

            /// Vertices
            // Get all three vertices
            std::vector<TriMesh::VertexHandle> vertexHandles;
            TriMesh::FaceVertexIter fv_it(mesh, faceh);
            for (; fv_it.is_valid(); ++fv_it) {
               vertexHandles.push_back(*fv_it);
            }

            // Find closest Vertex
            std::vector<TriMesh::VertexHandle>::iterator v_it(vertexHandles.begin());
            TriMesh::VertexHandle vertex(*v_it++);
            double min_v_dist = (mesh.point(vertex) - hit_point).norm();
            for (; v_it != vertexHandles.end(); ++v_it) {
               double v_dist = (mesh.point(*v_it) - hit_point).norm();
               if (v_dist < min_v_dist) {
                  min_v_dist = v_dist;
                  vertex = *v_it;
               }
            }
            active_vertex_ = vertex.idx();
         }
         // In the case of a polymesh
         else if (object->picked(node_idx) && object->dataType(DATA_POLY_MESH)) {
            PolyMesh& mesh = *PluginFunctions::polyMesh(object);
            PolyMesh::FaceHandle faceh = mesh.face_handle(target_idx);
            active_face_ = target_idx;

            // Get all halfedges on the face
            PolyMesh::FaceHalfedgeIter fhedge_it(mesh, faceh);
            std::vector<PolyMesh::HalfedgeHandle> halfEdgeHandles;
            for (;fhedge_it.is_valid(); ++fhedge_it) {
               halfEdgeHandles.push_back(*fhedge_it);
            }

            /// Edges
            // Find nearest edge
            PolyMesh::EdgeHandle edge = mesh.edge_handle(*halfEdgeHandles.begin());
            double min_dist = ACG::Geometry::distPointLineSquared(
                     hit_point,
                     mesh.point(mesh.to_vertex_handle(*halfEdgeHandles.begin())),
                     mesh.point(mesh.from_vertex_handle(*halfEdgeHandles.begin())));

            std::vector<PolyMesh::HalfedgeHandle>::iterator iter = halfEdgeHandles.begin();
            for (; iter != halfEdgeHandles.end(); ++iter) {
               double dist = ACG::Geometry::distPointLineSquared(
                        hit_point,
                        mesh.point(mesh.to_vertex_handle(*iter)),
                        mesh.point(mesh.from_vertex_handle(*iter)));

               if (dist < min_dist) {
                  edge = mesh.edge_handle(*iter);
                  min_dist = dist;
               }
            }
            active_edge_ = edge.idx();

            /// Vertices
            // Get all vertices
            std::vector<PolyMesh::VertexHandle> vertexHandles;
            PolyMesh::FaceVertexIter fv_it(mesh, faceh);
            for (; fv_it.is_valid(); ++fv_it) {
               vertexHandles.push_back(*fv_it);
            }

            // Find closest Vertex
            std::vector<PolyMesh::VertexHandle>::iterator v_it(vertexHandles.begin());
            PolyMesh::VertexHandle vertex(*v_it++);
            double min_v_dist = (mesh.point(vertex) - hit_point).norm();
            for (; v_it != vertexHandles.end(); ++v_it) {
               double v_dist = (mesh.point(*v_it) - hit_point).norm();
               if (v_dist < min_v_dist) {
                  min_v_dist = v_dist;
                  vertex = *v_it;
               }
            }
            active_vertex_ = vertex.idx();
         }
      }

      return object;
   }

   // Reset defaults
   active_face_ = -1;
   active_edge_ = -1;
   active_vertex_ = -1;
   active_hit_point_ = ACG::Vec3d(0.0);

   return 0;
}

/** \brief Find the selected edge and introduce a cut
 * @param _event the mouse click
 */
void MeshCut::singleEdgeCut(QMouseEvent* _event) {
   if (_event->type() != QEvent::MouseButtonPress)
      return;

   BaseObjectData* object = setActiveElements(_event);
   if (!object)
      return;

   if (object->dataType(DATA_TRIANGLE_MESH)) {
      TriMesh& mesh = *PluginFunctions::triMesh(object);

      // Cut edge
      if (cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
         // This is to show the result of selecting and cutting directly
         mesh.status(mesh.edge_handle(active_edge_)).set_selected(true);

         emit log(LOGOUT, "Cut edge " + QString::number(active_edge_));
         emit updatedObject(object->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(object->id(), "Edge cut", UPDATE_TOPOLOGY);
      }

   } else if (object->dataType(DATA_POLY_MESH)) {
      PolyMesh& mesh = *PluginFunctions::polyMesh(object);

      // Cut edge
      if (cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
         // This is to show the result of selecting and cutting directly
         mesh.status(mesh.edge_handle(active_edge_)).set_selected(true);

         emit log(LOGOUT, "Picked edge " + QString::number(active_edge_));
         emit updatedObject(object->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(object->id(), "Edge cut", UPDATE_TOPOLOGY);
      }
   }
}

/** \brief Edge selection
 * Sets the clicked edge as selected, or, if it already was, deselect it.
 *
 * @param _event The mouse click
 */
void MeshCut::selectEdge(QMouseEvent* _event) {
   if (_event->type() != QEvent::MouseButtonPress)
      return;

   BaseObjectData* object = setActiveElements(_event);
   if (!object)
      return;

   if (object->dataType(DATA_TRIANGLE_MESH)) {
      TriMesh& mesh = *PluginFunctions::triMesh(object);

      // Toggle edge selection
      TriMesh::EdgeHandle eh = mesh.edge_handle(active_edge_);
      mesh.status(eh).set_selected(!mesh.status(eh).selected());

      const char* prefix = mesh.status(eh).selected() ? "S" : "Des";
      emit log(LOGOUT, QString::fromStdString(prefix) + "elected TriMesh edge " + QString::number(active_edge_));
      emit updatedObject(object->id(), UPDATE_SELECTION_EDGES);
      emit updateView();
      emit createBackup(object->id(), "Edge cut", UPDATE_SELECTION_EDGES);

   } else if (object->dataType(DATA_POLY_MESH)) {
      emit log(LOGWARN, "Polymesh data structure not yet supported.");
      PolyMesh& mesh = *PluginFunctions::polyMesh(object);

      // Toggle edge selection
      PolyMesh::EdgeHandle eh = mesh.edge_handle(active_edge_);
      mesh.status(eh).set_selected(!mesh.status(eh).selected());

      const char* prefix = mesh.status(eh).selected() ? "S" : "Des";
      emit log(LOGOUT, QString::fromStdString(prefix) + "elected PolyMesh edge " + QString::number(active_edge_));
      emit updatedObject(object->id(), UPDATE_SELECTION_EDGES);
      emit updateView();
      emit createBackup(object->id(), "Edge cut", UPDATE_SELECTION_EDGES);
   }
}

/** \brief Draw a line and select the created edges
 *
 * At each edge crossing, save the current state of component variables,
 * perform a split and selection (if necessary), and advance.
 *
 * @param _event The mouse click
 */
void MeshCut::drawLine(QMouseEvent* _event) {
   if (_event->type() == QEvent::MouseButtonPress) {
      setActiveElements(_event);

      // Save click region and location
      prev_face_ = active_face_;
      prev_hit_point_ = ACG::Vec3d(active_hit_point_);

   } else if (_event->type() == QEvent::MouseMove) {
      BaseObjectData* object = setActiveElements(_event);

      /// TODO: cannot go out of mesh at the moment...
      // In case the mouse got out of the mesh
      bool gotOut = false;

      if (object) {
         // Save current elements while we can
         latest_object_ = object;
         latest_edge_ = active_edge_;
      } else if (latest_object_) {
         // Simulate the fact that we are still on the mesh and restore latest elements
         object = latest_object_;
         active_edge_ = latest_edge_;
         // We can restore from prev_hit_point_ because it was last updated before
         // getting out of the mesh. After that, we don't need it anymore.
         active_hit_point_ = prev_hit_point_;
         gotOut = true;
      }

      /// Triangular mesh
      if (object && object->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);

         // Keep track of the first edge before it is crossed
         if (active_edge_crossing_ == ACG::Vec3d(0.0)) {
            prev_edge_ = active_edge_;
         }

         // Face has changed or mouse got out, record edge crossing
         if (prev_face_ != active_face_ || gotOut) {  /// TODO when coming back to the previous face, be careful with the new index
            TriMesh::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(active_edge_), 0);

            /// Record crossing
            // Mouse is still over the mesh and crosses an edge
            if (prev_face_ != -1 && active_face_ != -1) {
               // Closest point on closest edge
               TriMesh::Point edge_point0 = ACG::Geometry::projectToEdge(
                        mesh.point(mesh.from_vertex_handle(heh)),
                        mesh.point(mesh.to_vertex_handle(heh)),
                        prev_hit_point_);

               TriMesh::Point edge_point1 = ACG::Geometry::projectToEdge(
                        mesh.point(mesh.from_vertex_handle(heh)),
                        mesh.point(mesh.to_vertex_handle(heh)),
                        active_hit_point_);

               // The crossing is defined as the middle point of the two closest edge projections
               active_edge_crossing_ = ACG::Vec3d((edge_point0 + edge_point1) / 2.0);
            }
            // Mouse is now out of the mesh
            else if (gotOut) {
               TriMesh::Point edge_point0 = ACG::Geometry::projectToEdge(
                        mesh.point(mesh.from_vertex_handle(heh)),
                        mesh.point(mesh.to_vertex_handle(heh)),
                        active_hit_point_);

               active_edge_crossing_ = ACG::Vec3d(edge_point0);
            }
            // Mouse came inside the mesh
            else if (active_face_ != -1){
               TriMesh::Point edge_point1 = ACG::Geometry::projectToEdge(
                        mesh.point(mesh.from_vertex_handle(heh)),
                        mesh.point(mesh.to_vertex_handle(heh)),
                        active_hit_point_);

               active_edge_crossing_ = ACG::Vec3d(edge_point1);
            }

            /// Split at previous crossing
            if (prev_edge_crossing_ != ACG::Vec3d(0.0) &&
                prev_edge_crossing_ != active_edge_crossing_) {
               TriMesh::VertexHandle vh = mesh.split(mesh.edge_handle(prev_edge_), prev_edge_crossing_);
               prev_edge_ = active_edge_;

               // Newly created vertex at edge split
               active_vertex_at_split_ = vh.idx();

               /// Select edge between current split and previous split
               if (prev_vertex_at_split_ != -1 &&
                   prev_vertex_at_split_ != active_vertex_at_split_) {  /// TODO: should we select edge at border?
                  // Select edge between the two vertices
                  int edge_to_select = edge_between(mesh.vertex_handle(prev_vertex_at_split_),
                                                   mesh.vertex_handle(active_vertex_at_split_),
                                                   mesh);
                  if (edge_to_select != -1) {
                     mesh.status(mesh.edge_handle(edge_to_select)).set_selected(true);
                  }
                  /* No need to insist if the cursor came out of the mesh
                   * and came back on another face */
                  else if (!gotOut) {
                     /// TODO: if mouse was too quick, the path still has to be created
                     emit log(LOGWARN, "Warning! The mouse was too quick and the cut was not correctly registered.");
                  }
               }

               // Save current vertex at split
               prev_vertex_at_split_ = active_vertex_at_split_;
            }

            // Save current edge crossing
            prev_edge_crossing_ = ACG::Vec3d(active_edge_crossing_);

            // Now update previous edge
            if (active_edge_ != -1)
               prev_edge_ = active_edge_;

            emit updatedObject(object->id(), UPDATE_TOPOLOGY);
            emit updateView();
         }

      }
      /// Polygonal mesh
      else if (object && object->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(object);
         emit log(LOGWARN, "Polymesh not yet supported for drawing cuts");
      }

      // In the end, save active face and hit point
      prev_face_ = active_face_;
      prev_hit_point_ = ACG::Vec3d(active_hit_point_);

   } else if (_event->type() == QEvent::MouseButtonRelease) {
      BaseObjectData* object = setActiveElements(_event);

      // In case the mouse got out of the mesh
      if (!object) {
         object = latest_object_;
      }

      if (object && object->dataType(DATA_TRIANGLE_MESH) &&
          prev_edge_crossing_ != ACG::Vec3d(0.0) && active_vertex_at_split_ != -1) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);

         // Finish selection
         TriMesh::VertexHandle vh = mesh.split(mesh.edge_handle(prev_edge_), prev_edge_crossing_);
         int edge_to_select = edge_between(mesh.vertex_handle(prev_vertex_at_split_), vh, mesh);
         if (edge_to_select != -1) {
            mesh.status(mesh.edge_handle(edge_to_select)).set_selected(true);
         } else {
            emit log(LOGWARN, "Warning! The mouse was too quick and the end of the cut was not correctly registered.");
         }

         emit log(LOGOUT, "Drew TriMesh cut");
         emit updatedObject(object->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(object->id(), "Selected drawing edges", UPDATE_TOPOLOGY);

      } else if (object && object->dataType(DATA_POLY_MESH)) {
         emit log(LOGWARN, "Polymesh not yet supported for drawing cuts");
      }

      // Reset all saved components
      latest_object_ = 0;
      latest_edge_ = -1;
      prev_face_ = -1;
      prev_edge_ = -1;
      prev_hit_point_ = ACG::Vec3d(0.0);
      active_edge_crossing_ = ACG::Vec3d(0.0);
      prev_edge_crossing_ = ACG::Vec3d(0.0);
      active_vertex_at_split_ = -1;
      prev_vertex_at_split_ = -1;
   }
}

/** \brief Cuts all selected edges
 * For all the meshes, look at all the selected edges and cut them
 * one by one.
 */
void MeshCut::slotCutSelectedEdges() {
   // Iterate over all objects
   for (PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS); o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         // Go through all edges
         size_t n_cuts(0);
         TriMesh::EdgeIter e_it, e_end(mesh.edges_end());
         for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (mesh.status(*e_it).selected() && cutPrimitive(*e_it, mesh)) {
               // Cut and count
               ++n_cuts;
            }
         }

         emit log(LOGOUT, "Cut " + QString::number(n_cuts) + " TriMesh edges");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected edges cut", UPDATE_TOPOLOGY);
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         emit log(LOGWARN, "No support for cutting polymeshes yet...");
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         // Iterate over all edges
         size_t n_cuts(0);
         PolyMesh::EdgeIter e_it, e_end(mesh.edges_end());
         for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (mesh.status(*e_it).selected() && cutPrimitive(*e_it, mesh)) {
               ++n_cuts;
            }
         }

         emit log(LOGOUT, "Cut " + QString::number(n_cuts) + " PolyMesh edges");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected edges cut", UPDATE_TOPOLOGY);
      }
   }
}

/** \brief Cut along edge
 *
 * For TriMesh only.
 * Look at the surrounding topology of the edge and cut along it.
 * This schematic shows how the middle edge relates to its
 * surroundings.
 *
 *              up vertex
 *                  x
 *                 /|\
 *               /  |  \
 *             /    |    \
 *           /      |      \
 *         /  left  | right  \
 *       x    face  |  face    x
 *         \        |        /
 *           \      |      /
 *             \    |    /
 *               \  |  /
 *                 \|/
 *                  x
 *             down vertex
 *
 * @param _edge The edge defining the cut
 * @param _mesh The mesh
 * @return success state
 */
template<typename MeshT>
bool MeshCut::cutPrimitive(typename MeshT::EdgeHandle edge, MeshT& mesh) {
   static_assert((std::is_same<MeshT, TriMesh>::value ||
                 std::is_same<MeshT, PolyMesh>::value),
                 "cutPrimitive: expected either TriMesh or PolyMesh");

   // Already a hole there
   if (mesh.is_boundary(edge)) {
      return false;
   }

   /// Get relevant neighbouring components
   // Original edge's halfedges
   typename MeshT::HalfedgeHandle heh_in = mesh.halfedge_handle(edge, 0);
   typename MeshT::HalfedgeHandle heh_out = mesh.halfedge_handle(edge, 1);

   // Left face halfedges
   typename MeshT::HalfedgeHandle left_up_heh_to_new = mesh.next_halfedge_handle(heh_out);
   typename MeshT::HalfedgeHandle left_down_heh_to_new = mesh.prev_halfedge_handle(heh_out);

   // Face adjacent to new edge
   typename MeshT::FaceHandle fh_left = mesh.face_handle(heh_out);

   // Edge endpoints
   typename MeshT::VertexHandle vh_up = mesh.from_vertex_handle(heh_in);
   typename MeshT::VertexHandle vh_down = mesh.to_vertex_handle(heh_in);


   /// Create new edge, copy all properties and adjust pointers
   // Outward new halfedge
   typename MeshT::HalfedgeHandle new_heh_out = mesh.new_edge(vh_down, vh_up);
   mesh.copy_all_properties(heh_in, new_heh_out, true);
   mesh.set_vertex_handle(new_heh_out, vh_down);
   mesh.set_next_halfedge_handle(new_heh_out, heh_out);
   mesh.set_next_halfedge_handle(heh_out, new_heh_out);
   // no need to set_prev_halfedge_handle as it is done in set_next_halfedge_handle
   mesh.set_halfedge_handle(vh_up, new_heh_out);

   // Inward new halfedge
   typename MeshT::HalfedgeHandle new_heh_in = mesh.opposite_halfedge_handle(new_heh_out);
   mesh.copy_all_properties(heh_out, new_heh_out, true);
   mesh.set_vertex_handle(new_heh_in, vh_up);
   mesh.set_face_handle(new_heh_in, fh_left);
   mesh.set_next_halfedge_handle(new_heh_in, left_up_heh_to_new);
   mesh.set_next_halfedge_handle(left_down_heh_to_new, new_heh_in);
   if (mesh.halfedge_handle(fh_left) == heh_out) {
      mesh.set_halfedge_handle(fh_left, new_heh_in);
   }
   mesh.set_halfedge_handle(vh_down, heh_out);

   // Outward halfedges are now boundary
   mesh.set_boundary(new_heh_out);
   mesh.set_boundary(heh_out);

   // If there are neighbouring cuts, connect them
   connectCuts(vh_up, mesh);
   connectCuts(vh_down, mesh);

   return true;
}

/** \brief Joins adjacent cuts
 *
 * If two adjacent cuts are detected, the edge will be split and the properties and pointers
 * of connected components will be updated.
 *
 *  Before:
 *                x          x
 *                |> cut  </
 *      left      |      /     right
 *      side      |    /      side
 *                |  /
 *                |/
 *     x----------x vh
 *                |\
 *                |  \
 *                |    \
 *                |      \
 *                |> cut  <\
 *                x          x
 *
 *  After:
 *                x             x
 *                |           /
 *      left      |>  cut  </     right
 *      side      |       /      side
 *                |     /
 *                |   /
 *     x----------x  x vh
 *         new_vh |   \
 *                |     \
 *                |       \
 *                |         \
 *                |           \
 *                x             x
 *
 * @param vh The connecting vertex
 * @param mesh The mesh
 */
template<typename MeshT>
void MeshCut::connectCuts(typename MeshT::VertexHandle vh, MeshT& mesh) {
   static_assert((std::is_same<MeshT, TriMesh>::value ||
                 std::is_same<MeshT, PolyMesh>::value),
                 "connectCuts: expected either TriMesh or PolyMesh");

   /// The new vertex will carry the left side edges
   const size_t _max_cuts = 2;
   // Halfedges at cuts
   typename MeshT::HalfedgeHandle heh_cuts[2*_max_cuts];
   // Halfedges on the left side of the cut
   std::vector<typename MeshT::HalfedgeHandle> left_halfedges;
   bool left_side = true;
   size_t cut = 0;

   // Count cuts and store halfedges according to side
   for (typename MeshT::VertexIHalfedgeIter vih_it = mesh.vih_iter(vh); vih_it.is_valid(); ++vih_it) {
      typename MeshT::HalfedgeHandle heh = *vih_it;
      if (mesh.is_boundary(heh) && cut < _max_cuts) {
         heh_cuts[2*cut] = heh;
         heh_cuts[2*cut+1] = mesh.next_halfedge_handle(heh);
         ++cut;
         left_side = !left_side;
      } else if (left_side) {
         left_halfedges.push_back(heh);
      }
   }

   // Number of cuts must not exceed _max_cuts
   if (cut > _max_cuts) {
      emit log(LOGERR, "Topology error, a vertex is connected to " + QString::number(cut) + " cuts");
   } else if (cut == _max_cuts) {
      // Split vertex
      typename MeshT::VertexHandle new_vh = mesh.add_vertex(mesh.point(vh));
      mesh.copy_all_properties(vh, new_vh, true);

      /// Update pointers
      // Left vertex boundary halfedges
      mesh.set_next_halfedge_handle(heh_cuts[0], heh_cuts[3]);
      mesh.set_halfedge_handle(new_vh, heh_cuts[3]);
      mesh.set_vertex_handle(heh_cuts[0], new_vh);
      // Other left halfedges
      if (!left_halfedges.empty()) {
         typename std::vector<typename MeshT::HalfedgeHandle>::iterator heh_it = left_halfedges.begin();
         for (; heh_it != left_halfedges.end(); ++heh_it) {
            mesh.set_vertex_handle(*heh_it, new_vh);
         }
      }

      // Right vertex
      mesh.set_next_halfedge_handle(heh_cuts[2], heh_cuts[1]);
      mesh.set_halfedge_handle(vh, heh_cuts[1]);
      mesh.set_vertex_handle(heh_cuts[2], vh);
      // No need to rewire right side halfedges
   }
}

/** \brief Get the index of the edge between two given vertices
 *
 * @param vh_from the first vertex
 * @param vh_to the second vertex
 * @param mesh the mesh on which reside the vertices and edge
 * @return the index of the edge
 */
template<typename MeshT>
int MeshCut::edge_between(typename MeshT::VertexHandle vh_from,
                          typename MeshT::VertexHandle vh_to, MeshT& mesh) {
   typename MeshT::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh_from);
   for (; voh_it.is_valid(); ++voh_it) {
      if (mesh.to_vertex_handle(*voh_it) == vh_to) {
         return mesh.edge_handle(*voh_it).idx();
      }
   }
   return -1;
}














