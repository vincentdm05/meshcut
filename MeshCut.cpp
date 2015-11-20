#include "MeshCut.hh"
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#define EDGE_CUT_POPUP "<B>Edge Cut</B><br>Cut along selected edge"
#define SELECT_EDGES_PICKMODE "MeshCut: Edge select"
#define DRAW_CUT_PICKMODE "MeshCut: Draw cut"

Q_EXPORT_PLUGIN2( meshCut , MeshCut );

MeshCut::MeshCut() :
   toolBar_(0), edgeCutAction_(0), toolBox_(0), selectButton_(0), drawButton_(0),
   clampToEdgeCheckBox_(0), directCutCheckBox_(0), selectionButtonToggled_(0),
   strainWeightSpinBox_(), cutting_tools_(),
   active_hit_point_(0.0), active_face_(-1), active_edge_(-1), active_vertex_(-1),
   visible_path_(), latest_object_(0),
   shape_tools_(), object_updated_(false) {}

/** \brief Initialize plugin
 *
 */
void MeshCut::initializePlugin()
{
   toolBox_ = new QWidget();
   QVBoxLayout* mainToolboxLayout = new QVBoxLayout(toolBox_);


   /// Cutting tools
   cutting_tools_ = new Cutting();

   QFont titleLabelFont("Arial", 15, QFont::Bold);
   QLabel* cuttingLabel = new QLabel("Cutting tools");
   cuttingLabel->setFont(titleLabelFont);
   mainToolboxLayout->addWidget(cuttingLabel);

   QHBoxLayout* toggleLayout = new QHBoxLayout(toolBox_);
   toggleLayout->setSpacing(5);

   selectButton_ = new QPushButton("&Select Edges", toolBox_);
   selectButton_->setToolTip("Select edges to be cut");
   selectButton_->setCheckable(true);
   toggleLayout->addWidget(selectButton_);

   drawButton_ = new QPushButton("&Draw Curve", toolBox_);
   drawButton_->setToolTip("Draw a path to be cut");
   drawButton_->setCheckable(true);
   toggleLayout->addWidget(drawButton_);

   mainToolboxLayout->addItem(toggleLayout);

   clampToEdgeCheckBox_ = new QCheckBox("Clamp drawn curve to mesh edges");
   mainToolboxLayout->addWidget(clampToEdgeCheckBox_);

   /// TODO: make this possible first
   directCutCheckBox_ = new QCheckBox("Cut directly as the mouse clicks");
//   mainToolboxLayout->addWidget(directCutCheckBox_);

   QPushButton* cutButton = new QPushButton("&Cut selected", toolBox_);
   cutButton->setToolTip("Cut selected edges.");
   mainToolboxLayout->addWidget(cutButton);

   // Add a line separator between tools
   mainToolboxLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Fixed));
   QFrame* lineSeparator = new QFrame();
   lineSeparator->setFrameShape(QFrame::HLine);
   lineSeparator->setFrameShadow(QFrame::Sunken);
   mainToolboxLayout->addWidget(lineSeparator);

   /// Shape tools
   shape_tools_ = new ShapeTools();

   QLabel* shapeLabel = new QLabel("Shape tools");
   shapeLabel->setFont(titleLabelFont);
   mainToolboxLayout->addWidget(shapeLabel);

   // Fixed vertices toggle
   QPushButton* toggleFixSelectedButton = new QPushButton("Fix/unfix selected vertices", toolBox_);
   toggleFixSelectedButton->setToolTip("Toggle constraint to fix selected vertices to their current position");
   mainToolboxLayout->addWidget(toggleFixSelectedButton);

   // Strain weight
   QHBoxLayout* strainWeightLayout = new QHBoxLayout(toolBox_);
   strainWeightLayout->setSpacing(5);

   QLabel* strainWeightLabel = new QLabel(toolBox_);
   strainWeightLabel->setText(QObject::tr("Edge strain weight"));
   strainWeightLabel->setAlignment(Qt::AlignLeft);
   strainWeightLayout->addWidget(strainWeightLabel);

   strainWeightSpinBox_ = new QSpinBox(toolBox_);
   strainWeightSpinBox_->setMinimum(0);
   strainWeightSpinBox_->setMaximum(50);
   strainWeightSpinBox_->setSingleStep(1);
   strainWeightSpinBox_->setValue(10);
   strainWeightLayout->addWidget(strainWeightSpinBox_);

   mainToolboxLayout->addItem(strainWeightLayout);

   // Play/pause button
   QPushButton* playPauseShapeToolButton = new QPushButton("Play/Pause shape solving", toolBox_);
   playPauseShapeToolButton->setToolTip("Play and pause shape solving actions");
   playPauseShapeToolButton->setCheckable(true);
   mainToolboxLayout->addWidget(playPauseShapeToolButton);

   // Spacer at the end
   mainToolboxLayout->addItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Fixed));

   /// Connect signals->slots
   connect(selectButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(drawButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(cutButton, SIGNAL(clicked()), this, SLOT(slotCutSelectedEdges()));
   connect(toggleFixSelectedButton, SIGNAL(clicked()), this, SLOT(slotFixSelectedVertices()));
   connect(playPauseShapeToolButton, SIGNAL(clicked()), this, SLOT(slotPlayPauseShapeTools()));

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
   } else if (shape_tools_->isRunning() && PluginFunctions::pickMode() == "MoveSelection" &&
              PluginFunctions::actionMode() == Viewer::PickingMode) {
      moveMesh(_event);
   }
}

/**
 * @brief MeshCut::slotObjectUpdated
 * @param _id The id of the updated object
 * @param _type
 */
void MeshCut::slotObjectUpdated(int _id, const UpdateType& _type) {
   if (_id == shape_tools_->getObjId() && _type == UPDATE_GEOMETRY) {
      emit log(LOGOUT, "object updated");
      object_updated_ = true;
//      shape_tools_->updateMesh();
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
      if (cutting_tools_->cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
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
      if (cutting_tools_->cutPrimitive(mesh.edge_handle(active_edge_), mesh)) {
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
         if (cutting_tools_->pathSize() > 1) {
            showPath(std::get<TUPLE_POINT>(cutting_tools_->getPathPoint(PATH_BACK)), active_hit_point_, object);
         }
         cutting_tools_->addPathPoint(std::make_tuple(active_hit_point_, active_face_, active_edge_, active_vertex_));

         // If real time cutting is selected, apply after two face-overs
         if (directCutCheckBox_->isChecked() && cutting_tools_->faceOvers() > 2) {
            /// TODO: real time cutting is hardcore
            emit log(LOGWARN, "Real-time cutting is still under development.");
         }
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

/** \brief Solve and update positions of vertices on mesh
 *
 * @param _event
 */
void MeshCut::moveMesh(QMouseEvent *_event) {
   emit log(LOGOUT, "move mesh");
   if (_event->type() == QEvent::MouseButtonPress) {
      TriMesh* mesh;
      PluginFunctions::getMesh(shape_tools_->getObjId(), mesh);

      // Find handle index
      std::vector<int> handle_idxs;
      TriMesh::VertexIter v_it(mesh->vertices_begin());
      for (; v_it!=mesh->vertices_end(); ++v_it) {
         if (mesh->status(*v_it).selected()) {
            handle_idxs.push_back((*v_it).idx());
         }
      }

      shape_tools_->setHandles(handle_idxs);
      shape_tools_->setEdgeStrain(strainWeightSpinBox_->value()/ 10.0);
   } else if (object_updated_ && _event->type() == QEvent::MouseMove) {
      shape_tools_->solveUpdateMesh();
      emit updatedObject(shape_tools_->getObjId(), UPDATE_GEOMETRY);
      emit updateView();
      object_updated_ = false;
   } else if (_event->type() == QEvent::MouseButtonRelease) {
      shape_tools_->setHandles(std::vector<int>());
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
   if (!object || cutting_tools_->pathEmpty()) return;

   // Apply to mesh
   if (clampToEdgeCheckBox_->isChecked()) {
      if (object->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);
         cutting_tools_->clampAndSelect(mesh);
      } else if (object->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(object);
         cutting_tools_->clampAndSelect(mesh);
      }
   } else {
      cutting_tools_->markForSplit(object);
      if (object->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);
         cutting_tools_->splitAndSelect(mesh);
      } else if (object->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(object);
         cutting_tools_->splitAndSelect(mesh);
      }
   }

   // Remove drawn curve
   for (std::vector<ACG::SceneGraph::LineNode*>::iterator it = visible_path_.begin(); it != visible_path_.end(); ++it) {
      delete *it;
   }
   visible_path_.clear();

   // Reset global variables
   latest_object_ = 0;
   cutting_tools_->clearPath();

   // Notify OpenFlipper of changes
   emit log(LOGOUT, "Drew curve on mesh");
   emit updatedObject(object->id(), UPDATE_TOPOLOGY);
   emit updateView();
   emit createBackup(object->id(), "Path application", UPDATE_TOPOLOGY);
}

/** \brief Cuts all selected edges
 * For all the meshes, look at all the selected edges and cut them
 * one by one.
 */
void MeshCut::slotCutSelectedEdges() {
   // Iterate over all objects
   PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         // Go through all edges
         size_t n_cuts(0);
         TriMesh::EdgeIter e_it, e_end(mesh.edges_end());
         for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (mesh.status(*e_it).selected() && cutting_tools_->cutPrimitive(*e_it, mesh)) {
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
            if (mesh.status(*e_it).selected() && cutting_tools_->cutPrimitive(*e_it, mesh)) {
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

/** \brief Toggle closenessConstraint on selected vertices
 *
 */
void MeshCut::slotFixSelectedVertices() {
   std::set<int> v_idxs;
   PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      v_idxs.clear();

      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);
         shape_tools_->setMesh(&mesh, o_it->id());

         TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               v_idxs.insert((*v_it).idx());
            }
         }

         shape_tools_->toggleFixVertices(v_idxs);
      }

      /// At the moment, only one object is supported
      break;
   }

   emit updatedObject(shape_tools_->getObjId(), UPDATE_SELECTION_VERTICES);
   emit updatedObject(shape_tools_->getObjId(), UPDATE_COLOR);
   emit updateView();
}

/** \brief Toggle play/pause in shape tools
 *
 */
void MeshCut::slotPlayPauseShapeTools() {
   shape_tools_->playPause();
}













