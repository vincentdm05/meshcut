#include "MeshCut.hh"
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#define EDGE_CUT_POPUP "<B>Edge Cut</B><br>Cut along selected edge"
#define SELECT_EDGES_PICKMODE "MeshCut: Edge select"
#define DRAW_CUT_PICKMODE "MeshCut: Draw cut"

Q_EXPORT_PLUGIN2( meshCut , MeshCut );

MeshCut::MeshCut() :
   toolBar_(0), edgeCutAction_(0), toolBox_(0), selectButton_(0), drawButton_(0), selectionButtonToggled_(0),
   cutting_(0),
   active_hit_point_(0.0), active_face_(-1), active_edge_(-1), active_vertex_(-1),
   visible_path_(), recorded_path_(), edges_to_split_(), latest_object_(0) {}

/** \brief Initialize plugin
 *
 */
void MeshCut::initializePlugin()
{
   // Create cutting tools
   cutting_ = new Cutting();

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
      mouseDraw(_event);
   }
}

/** \brief Sets closest elements as active
 * There are four elements that are set as active here, namely the index of the
 * closest face, edge and vertex, and the hit point.
 * The object container is returned so that we can access the mesh.
 *
 * @param _event the mouse click event
 */
BaseObjectData *MeshCut::setActiveElements(QPoint _pos) {
   unsigned int node_idx, target_idx;
   ACG::Vec3d hit_point;
   if (PluginFunctions::scenegraphPick(ACG::SceneGraph::PICK_FACE, _pos, node_idx, target_idx, &hit_point)) {
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

/** \brief Cut the clicked edge
 * @param _event the mouse click
 */
void MeshCut::singleEdgeCut(QMouseEvent* _event) {
   if (_event->type() != QEvent::MouseButtonPress)
      return;

   BaseObjectData* object = setActiveElements(_event->pos());
   if (!object)
      return;

   if (object->dataType(DATA_TRIANGLE_MESH)) {
      TriMesh& mesh = *PluginFunctions::triMesh(object);

      // Cut edge
      if (cutting_->cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
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
      if (cutting_->cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
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

   BaseObjectData* object = setActiveElements(_event->pos());
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
void MeshCut::mouseDraw(QMouseEvent* _event) {
   if (_event->type() == QEvent::MouseMove) {
      BaseObjectData* object = setActiveElements(_event->pos());
      if (object) {
         // Save last seen object
         latest_object_ = object;

         // Show path and record it
         if (recorded_path_.size() > 1)
            showPath(std::get<TUPLE_POINT>(recorded_path_.back()), active_hit_point_, object);
         recorded_path_.push_back(std::make_tuple(active_hit_point_, active_face_, active_edge_));
      }
      // The mouse got out of the mesh
      else if (!visible_path_.empty()) {
         applyCurve(latest_object_);
      }

   } else if (_event->type() == QEvent::MouseButtonRelease) {
      BaseObjectData* object = setActiveElements(_event->pos());

      if (!object) {
         // Reset last active object
         object = latest_object_;
      }

      if (object && !visible_path_.empty()) {
         applyCurve(object);
      }
   }
}

/** \brief Show consecutive hit points as path
 * Make the current mouse path visible by creating lines between selected recorded hit points.
 */
void MeshCut::showPath(ACG::Vec3d _prev_point, ACG::Vec3d _curr_point, BaseObjectData* object) {
   if (!object) return;

   ACG::SceneGraph::LineNode* line = new ACG::SceneGraph::LineNode(
            ACG::SceneGraph::LineNode::LineSegmentsMode, object->manipulatorNode());
   line->set_color(OpenMesh::Vec4f(1.0f,0.0f,0.0f,1.0f));
   line->set_line_width(2);
   line->add_line(_prev_point, _curr_point);
   line->alwaysOnTop() = true;

   visible_path_.push_back(line);

   emit updatedObject(object->id(), UPDATE_TOPOLOGY);
   emit updateView();
}

/** \brief Finalize mouse interaction
 * Apply curve to mesh, then remove visible path and reset variables.
 *
 * @param object The object on which to apply a path
 */
void MeshCut::applyCurve(BaseObjectData *object) {
   if (!object) return;

   // Apply to mesh
   markForSplit(object);
   cutting_->setEdgesToSplit(&edges_to_split_);
   if (object->dataType(DATA_TRIANGLE_MESH)) {
      TriMesh& mesh = *PluginFunctions::triMesh(object);
      cutting_->splitAndSelect(mesh);
   } else if (object->dataType(DATA_POLY_MESH)) {
      PolyMesh& mesh = *PluginFunctions::polyMesh(object);
      cutting_->splitAndSelect(mesh);
   }

   // Clear visible path variable
   for (std::vector<ACG::SceneGraph::LineNode*>::iterator it = visible_path_.begin(); it != visible_path_.end(); ++it) {
      delete *it;
   }
   visible_path_.clear();

   // Reset global variables
   latest_object_ = 0;
   recorded_path_.clear();
//   while (!recorded_path_.empty()) {
//      recorded_path_.pop_front(); /// TODO: Should we remove the last elements? or allow to connect to the next path?
//   }

   // Notify OpenFlipper of changes
   emit log(LOGOUT, "Drew curve on mesh");
   emit updatedObject(object->id(), UPDATE_TOPOLOGY);
   emit updateView();
   emit createBackup(object->id(), "Path application", UPDATE_TOPOLOGY);
}

/** \brief Follow path and mark edges for split
 * Edges crossed by the mouse path are stored with the crossing point for later splitting.
 */
void MeshCut::markForSplit(BaseObjectData* object) {
   if (recorded_path_.size() < 2 || !object) return;

   while(!recorded_path_.empty()) {
      // Advance until there is a change of face
      std::tuple<ACG::Vec3d,int,int> prev_point = recorded_path_.front();
      recorded_path_.pop_front();
      std::tuple<ACG::Vec3d,int,int> curr_point = recorded_path_.front();
      while (!recorded_path_.empty() && std::get<TUPLE_FACE>(curr_point) == std::get<TUPLE_FACE>(prev_point)) {
         prev_point = curr_point;
         recorded_path_.pop_front();
         curr_point = recorded_path_.front();
      }
      if (recorded_path_.empty()) return;

      // Get the two points and their closest edge
      ACG::Vec3d prev_hit_point = std::get<TUPLE_POINT>(prev_point);
      int prev_edge = std::get<TUPLE_EDGE>(prev_point);
      ACG::Vec3d curr_hit_point = std::get<TUPLE_POINT>(curr_point);
      int curr_edge = std::get<TUPLE_EDGE>(curr_point);

      /// Triangle mesh
      if (object->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);

         /// Find crossing(s) and mark for split
         // Points are on both sides of the same edge
         if (prev_edge == curr_edge) {
            TriMesh::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(curr_edge), 0);

            // Vector from previous to current point
            ACG::Vec3d prev_to_curr = curr_hit_point - prev_hit_point;

            // Distance from previous point to edge
            double prev_point_edge_dist = ACG::Geometry::distPointLine(prev_hit_point,
                     mesh.point(mesh.from_vertex_handle(heh)),
                     mesh.point(mesh.to_vertex_handle(heh)));

            // Distance from current point to edge
            double curr_point_edge_dist = ACG::Geometry::distPointLine(curr_hit_point,
                     mesh.point(mesh.from_vertex_handle(heh)),
                     mesh.point(mesh.to_vertex_handle(heh)));

            // Crossing point distance ratio
            double ratio = prev_point_edge_dist / (prev_point_edge_dist + curr_point_edge_dist);

            // Actual crossing point
            ACG::Vec3d edge_crossing(prev_hit_point + ratio * prev_to_curr);

            // Store edge crossing
            edges_to_split_.push(std::make_pair(curr_edge, edge_crossing));
         }
         // Points are further apart
         else {
            // Project end point to plane defined by triangle under first point
            TriMesh::Normal face_normal = mesh.calc_face_normal(mesh.face_handle(std::get<TUPLE_FACE>(prev_point)));
            ACG::Vec3d projected_end = ACG::Geometry::projectToPlane(prev_hit_point, face_normal, curr_hit_point);
            Eigen::Vector3d p0(prev_hit_point[0], prev_hit_point[1], prev_hit_point[2]);
            Eigen::Vector3d p1(projected_end[0],projected_end[1],projected_end[2]);

            // Find crossed edge and point
            Eigen::Vector3d intersection_point;
            int crossed_edge_idx = -1;
            TriMesh::HalfedgeHandle crossed_halfedge;
            TriMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(mesh.face_handle(std::get<TUPLE_FACE>(prev_point)));
            for (; fh_it; ++fh_it) {
               TriMesh::Point point_from = mesh.point(mesh.from_vertex_handle(*fh_it));
               Eigen::Vector3d q0(point_from[0], point_from[1], point_from[2]);
               TriMesh::Point point_to = mesh.point(mesh.to_vertex_handle(*fh_it));
               Eigen::Vector3d q1(point_to[0], point_to[1], point_to[2]);

               // Get intersection point
               if (cutting_->segmentIntersect(p0, p1, q0, q1, &intersection_point)) {
                  crossed_edge_idx = mesh.edge_handle(*fh_it).idx();
                  crossed_halfedge = *fh_it;
                  break;
               }
            }
            /// TODO: Sometimes an intersection cannot be found. Investigate whether it's because of collinearity
            /// or because it crossed exactly on a point. (which is probably equivalent in the problem)
            if (crossed_edge_idx == -1) {
               emit log(LOGERR, "Could not find segment intersection");
               continue;
            }

            // Mark edge and crossing point
            ACG::Vec3d crossing(intersection_point[0], intersection_point[1], intersection_point[2]);
            edges_to_split_.push(std::make_pair(crossed_edge_idx, crossing));


            // Project rest of line to next face
            TriMesh::FaceHandle next_face = mesh.face_handle(mesh.opposite_halfedge_handle(crossed_halfedge));
            TriMesh::Normal next_face_normal = mesh.calc_face_normal(next_face);
            Eigen::Vector3d n(next_face_normal[0], next_face_normal[1], next_face_normal[2]);
            p0 = Eigen::Vector3d(intersection_point);
            Eigen::Hyperplane<double,3> plane(n, p0);
            Eigen::Vector3d curr_p(curr_hit_point[0], curr_hit_point[1], curr_hit_point[2]);
            p1 = plane.projection(curr_p);

            // Place new point inside the triangle
            Eigen::Vector3d point_in_face;
            ACG::Vec3d face_centroid = mesh.calc_face_centroid(next_face);
            Eigen::Vector3d q0(face_centroid[0], face_centroid[1], face_centroid[2]);
            TriMesh::FaceVertexIter fv_it = mesh.fv_iter(next_face);
            // Intersect the line with segments going from face centroid to face vertices
            for (; fv_it; ++fv_it) {
               ACG::Vec3d edge_point = mesh.point(*fv_it);
               Eigen::Vector3d q1(edge_point[0], edge_point[1], edge_point[2]);

               // As soon as an edge is crossed, stop: we only need one
               if (cutting_->segmentIntersect(p0, p1, q0, q1, &point_in_face)) break;
            }

            // Add new point to recorded path with next face
            recorded_path_.push_front(std::make_tuple(ACG::Vec3d(point_in_face[0], point_in_face[1], point_in_face[2]),
                  next_face.idx(), crossed_edge_idx));
         }
      }
      /// Polygonal mesh
      else if (object->dataType(DATA_POLY_MESH)) {
         emit log(LOGWARN, "No support for polymesh yet");
         return;
      }
   }
}

/** \brief Cuts all selected edges
 * For all the meshes, look at all the selected edges and cut them
 * one by one.
 */
void MeshCut::slotCutSelectedEdges() {
   // Iterate over all objects
   PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         // Go through all edges
         size_t n_cuts(0);
         TriMesh::EdgeIter e_it, e_end(mesh.edges_end());
         for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (mesh.status(*e_it).selected() && cutting_->cutPrimitive(*e_it, mesh)) {
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
            if (mesh.status(*e_it).selected() && cutting_->cutPrimitive(*e_it, mesh)) {
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















