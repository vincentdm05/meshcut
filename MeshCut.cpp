#include "MeshCut.hh"
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#define EDGE_CUT_POPUP "<B>Edge Cut</B><br>Cut along selected edge"
#define SELECT_EDGES_PICKMODE "MeshCut: Edge select"
#define DRAW_CUT_PICKMODE "MeshCut: Draw cut"

Q_EXPORT_PLUGIN2( meshCut , MeshCut );

/** \brief Initialize plugin
 *
 */
void MeshCut::initializePlugin()
{
   toolBox_ = new QWidget();
   QVBoxLayout* mainToolboxLayout = new QVBoxLayout(toolBox_);
   QFont titleLabelFont("Arial", 15, QFont::Bold);

   ////////////////////////////////////////////////////////////////////////////////////////////////
   /// Cutting tools
   cutting_tools_ = new Cutting();

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

   clampToEdgeCheckBox_ = new QCheckBox("Clamp drawn curve to mesh edges", toolBox_);
   mainToolboxLayout->addWidget(clampToEdgeCheckBox_);

   /// TODO: make this possible first
//   directCutCheckBox_ = new QCheckBox("Cut directly as the mouse clicks", toolBox_);
//   mainToolboxLayout->addWidget(directCutCheckBox_);

   independantCutsCheckBox_ = new QCheckBox("Don't split vertices", toolBox_);
   mainToolboxLayout->addWidget(independantCutsCheckBox_);

   QPushButton* cutButton = new QPushButton("&Cut selected", toolBox_);
   cutButton->setToolTip("Cut selected edges.");
   mainToolboxLayout->addWidget(cutButton);

   QPushButton* attachButton = new QPushButton("&Attach selected", toolBox_);
   attachButton->setToolTip("Attach selected vertices together");
   mainToolboxLayout->addWidget(attachButton);

   QPushButton* splitVertexButton = new QPushButton("&Split selected vertices", toolBox_);
   splitVertexButton->setToolTip("Split vertices that are adjacent to two cuts");
   mainToolboxLayout->addWidget(splitVertexButton);

   // Line separator between tools
   mainToolboxLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Fixed));
   QFrame* lineSeparator0 = new QFrame();
   lineSeparator0->setFrameShape(QFrame::HLine);
   lineSeparator0->setFrameShadow(QFrame::Sunken);
   mainToolboxLayout->addWidget(lineSeparator0);

   ////////////////////////////////////////////////////////////////////////////////////////////////
   /// Shape tools
   shape_tools_ = new ShapeTools();

   // Tool title
   QLabel* shapeLabel = new QLabel("Shape tools");
   shapeLabel->setFont(titleLabelFont);
   mainToolboxLayout->addWidget(shapeLabel);

   // Shape tools usage toggle
   QCheckBox* useShapeToolsCheckBox = new QCheckBox("Use shape tools", toolBox_);
   mainToolboxLayout->addWidget(useShapeToolsCheckBox);

   // Fixed vertices button
   QPushButton* fixSelectedButton = new QPushButton("Fix selected vertices", toolBox_);
   fixSelectedButton->setToolTip("Set vertices to be fixed to their current position");
   mainToolboxLayout->addWidget(fixSelectedButton);

   // Rigid shape
   QPushButton* rigidifyButton = new QPushButton("Rigidify selected shape", toolBox_);
   rigidifyButton->setToolTip("Rigidify shape given by selected vertices");
   mainToolboxLayout->addWidget(rigidifyButton);

   /// Constraints
   // Titles
   QHBoxLayout* constraintTitlesLayout = new QHBoxLayout(toolBox_);
   constraintTitlesLayout->setSpacing(5);
   QLabel* constraintTypeLabel = new QLabel(toolBox_);
   constraintTypeLabel->setText("Constraint type");
   constraintTypeLabel->setAlignment(Qt::AlignLeft);
   constraintTitlesLayout->addWidget(constraintTypeLabel);
   QLabel* constraintMinLabel = new QLabel(toolBox_);
   constraintMinLabel->setText("Min");
   constraintMinLabel->setAlignment(Qt::AlignCenter);
   constraintTitlesLayout->addWidget(constraintMinLabel);
   QLabel* constraintMaxLabel = new QLabel(toolBox_);
   constraintMaxLabel->setText("Max");
   constraintMaxLabel->setAlignment(Qt::AlignCenter);
   constraintTitlesLayout->addWidget(constraintMaxLabel);
   QLabel* constraintWeightLabel = new QLabel(toolBox_);
   constraintWeightLabel->setText("Weight");
   constraintWeightLabel->setAlignment(Qt::AlignRight);
   constraintTitlesLayout->addWidget(constraintWeightLabel);
   mainToolboxLayout->addItem(constraintTitlesLayout);

   // Edge strain
   QHBoxLayout* edgeStrainLayout = new QHBoxLayout(toolBox_);
   edgeStrainLayout->setSpacing(5);
   edgeStrainCheckBox_ = new QCheckBox("Edge strain", toolBox_);
   edgeStrainLayout->addWidget(edgeStrainCheckBox_);
   edgeStrainLayout->addItem(new QSpacerItem(10, 0, QSizePolicy::Expanding, QSizePolicy::Expanding));
   edgeStrainWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   edgeStrainWeightSpinBox_->setMinimum(WEIGHT_MIN);
   edgeStrainWeightSpinBox_->setMaximum(WEIGHT_MAX);
   edgeStrainWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   edgeStrainWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   edgeStrainLayout->addWidget(edgeStrainWeightSpinBox_);
   mainToolboxLayout->addItem(edgeStrainLayout);

   // Triangle strain
   QHBoxLayout* triangleStrainLayout = new QHBoxLayout(toolBox_);
   triangleStrainLayout->setSpacing(5);
   triangleStrainCheckBox_ = new QCheckBox("Triangle strain", toolBox_);
   triangleStrainLayout->addWidget(triangleStrainCheckBox_);
   triangleStrainLayout->addItem(new QSpacerItem(10, 0, QSizePolicy::Expanding, QSizePolicy::Expanding));
   triangleStrainWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   triangleStrainWeightSpinBox_->setMinimum(WEIGHT_MIN);
   triangleStrainWeightSpinBox_->setMaximum(WEIGHT_MAX);
   triangleStrainWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   triangleStrainWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   triangleStrainLayout->addWidget(triangleStrainWeightSpinBox_);
   mainToolboxLayout->addItem(triangleStrainLayout);

   // Area constraint
   QHBoxLayout* areaConstraintLayout = new QHBoxLayout(toolBox_);
   areaConstraintLayout->setSpacing(5);
   areaConstraintCheckBox_ = new QCheckBox("Area", toolBox_);
   areaConstraintLayout->addWidget(areaConstraintCheckBox_);
   areaConstraintMinSpinBox_ = new QDoubleSpinBox(toolBox_);
   areaConstraintMinSpinBox_->setMinimum(RANGE_MIN);
   areaConstraintMinSpinBox_->setMaximum(RANGE_MAX);
   areaConstraintMinSpinBox_->setSingleStep(RANGE_STEP);
   areaConstraintMinSpinBox_->setValue(RANGE_DEFAULT);
   areaConstraintLayout->addWidget(areaConstraintMinSpinBox_);
   areaConstraintMaxSpinBox_ = new QDoubleSpinBox(toolBox_);
   areaConstraintMaxSpinBox_->setMinimum(RANGE_MIN);
   areaConstraintMaxSpinBox_->setMaximum(RANGE_MAX);
   areaConstraintMaxSpinBox_->setSingleStep(RANGE_STEP);
   areaConstraintMaxSpinBox_->setValue(RANGE_DEFAULT);
   areaConstraintLayout->addWidget(areaConstraintMaxSpinBox_);
   areaConstraintWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   areaConstraintWeightSpinBox_->setMinimum(WEIGHT_MIN);
   areaConstraintWeightSpinBox_->setMaximum(WEIGHT_MAX);
   areaConstraintWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   areaConstraintWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   areaConstraintLayout->addWidget(areaConstraintWeightSpinBox_);
   mainToolboxLayout->addItem(areaConstraintLayout);

   // Bending constraint
   QHBoxLayout* bendingConstraintLayout = new QHBoxLayout(toolBox_);
   bendingConstraintLayout->setSpacing(5);
   bendingConstraintCheckBox_ = new QCheckBox("Bending", toolBox_);
   bendingConstraintLayout->addWidget(bendingConstraintCheckBox_);
   bendingConstraintMinSpinBox_ = new QDoubleSpinBox(toolBox_);
   bendingConstraintMinSpinBox_->setMinimum(RANGE_MIN);
   bendingConstraintMinSpinBox_->setMaximum(RANGE_MAX);
   bendingConstraintMinSpinBox_->setSingleStep(RANGE_STEP);
   bendingConstraintMinSpinBox_->setValue(RANGE_DEFAULT);
   bendingConstraintLayout->addWidget(bendingConstraintMinSpinBox_);
   bendingConstraintMaxSpinBox_ = new QDoubleSpinBox(toolBox_);
   bendingConstraintMaxSpinBox_->setMinimum(RANGE_MIN);
   bendingConstraintMaxSpinBox_->setMaximum(RANGE_MAX);
   bendingConstraintMaxSpinBox_->setSingleStep(RANGE_STEP);
   bendingConstraintMaxSpinBox_->setValue(RANGE_DEFAULT);
   bendingConstraintLayout->addWidget(bendingConstraintMaxSpinBox_);
   bendingConstraintWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   bendingConstraintWeightSpinBox_->setMinimum(WEIGHT_MIN);
   bendingConstraintWeightSpinBox_->setMaximum(WEIGHT_MAX);
   bendingConstraintWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   bendingConstraintWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   bendingConstraintLayout->addWidget(bendingConstraintWeightSpinBox_);
   mainToolboxLayout->addItem(bendingConstraintLayout);

   // Rectangle constraint
   QHBoxLayout* rectConstraintLayout = new QHBoxLayout(toolBox_);
   rectConstraintLayout->setSpacing(5);
   rectConstraintCheckBox_ = new QCheckBox("Rectangle constraint", toolBox_);
   rectConstraintLayout->addWidget(rectConstraintCheckBox_);
   rectConstraintLayout->addItem(new QSpacerItem(10, 0, QSizePolicy::Expanding, QSizePolicy::Expanding));
   rectConstraintWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   rectConstraintWeightSpinBox_->setMinimum(WEIGHT_MIN);
   rectConstraintWeightSpinBox_->setMaximum(WEIGHT_MAX);
   rectConstraintWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   rectConstraintWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   rectConstraintLayout->addWidget(rectConstraintWeightSpinBox_);
   mainToolboxLayout->addItem(rectConstraintLayout);

   // Angle constraint
   QHBoxLayout* angleConstraintLayout = new QHBoxLayout(toolBox_);
   angleConstraintLayout->setSpacing(5);
   angleConstraintCheckBox_ = new QCheckBox("Angle", toolBox_);
   angleConstraintLayout->addWidget(angleConstraintCheckBox_);
   angleConstraintMinSpinBox_ = new QSpinBox(toolBox_);
   angleConstraintMinSpinBox_->setMinimum(0);
   angleConstraintMinSpinBox_->setMaximum(180);
   angleConstraintMinSpinBox_->setSingleStep(1);
   angleConstraintMinSpinBox_->setValue(0);
   angleConstraintLayout->addWidget(angleConstraintMinSpinBox_);
   angleConstraintMaxSpinBox_ = new QSpinBox(toolBox_);
   angleConstraintMaxSpinBox_->setMinimum(0);
   angleConstraintMaxSpinBox_->setMaximum(180);
   angleConstraintMaxSpinBox_->setSingleStep(1);
   angleConstraintMaxSpinBox_->setValue(60);
   angleConstraintLayout->addWidget(angleConstraintMaxSpinBox_);
   angleConstraintWeightSpinBox_ = new QDoubleSpinBox(toolBox_);
   angleConstraintWeightSpinBox_->setMinimum(WEIGHT_MIN);
   angleConstraintWeightSpinBox_->setMaximum(WEIGHT_MAX);
   angleConstraintWeightSpinBox_->setSingleStep(WEIGHT_STEP);
   angleConstraintWeightSpinBox_->setValue(WEIGHT_DEFAULT);
   angleConstraintLayout->addWidget(angleConstraintWeightSpinBox_);
   mainToolboxLayout->addItem(angleConstraintLayout);


   // Instantaneous update
   QHBoxLayout* updateNowLayout = new QHBoxLayout(toolBox_);
   updateNowLayout->setSpacing(5);
   QPushButton* updateButton = new QPushButton("Update now", toolBox_);
   updateNowLayout->addWidget(updateButton);
   nSolverIterationsSpinBox_ = new QSpinBox(toolBox_);
   nSolverIterationsSpinBox_->setMinimum(1);
   nSolverIterationsSpinBox_->setMaximum(100);
   nSolverIterationsSpinBox_->setSingleStep(1);
   nSolverIterationsSpinBox_->setValue(10);
   updateNowLayout->addWidget(nSolverIterationsSpinBox_);
   mainToolboxLayout->addItem(updateNowLayout);

   // Line separator between tools
   mainToolboxLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Fixed));
   QFrame* lineSeparator1 = new QFrame();
   lineSeparator1->setFrameShape(QFrame::HLine);
   lineSeparator1->setFrameShadow(QFrame::Sunken);
   mainToolboxLayout->addWidget(lineSeparator1);

   ////////////////////////////////////////////////////////////////////////////////////////////////
   /// Mesh generator
   mesh_generator_ = new MeshGen();

   // Tool title
   QLabel* meshGenLabel = new QLabel("Mesh Generator");
   meshGenLabel->setFont(titleLabelFont);
   mainToolboxLayout->addWidget(meshGenLabel);

   // Tessellation checkbox
   hingedTessellationCheckBox_ = new QCheckBox("Hinged tessellation", toolBox_);
   mainToolboxLayout->addWidget(hingedTessellationCheckBox_);

   // Quad generation
   QHBoxLayout* quadGenLayout = new QHBoxLayout(toolBox_);
   quadGenLayout->setSpacing(5);
   QPushButton* quadGenButton = new QPushButton("Generate quad mesh", toolBox_);
   quadGenLayout->addWidget(quadGenButton);
   quadGenWidthSpinBox_ = new QSpinBox(toolBox_);
   quadGenWidthSpinBox_->setMinimum(1);
   quadGenWidthSpinBox_->setMaximum(1000);
   quadGenWidthSpinBox_->setSingleStep(1);
   quadGenWidthSpinBox_->setValue(10);
   quadGenLayout->addWidget(quadGenWidthSpinBox_);
   quadGenHeightSpinBox_ = new QSpinBox(toolBox_);
   quadGenHeightSpinBox_->setMinimum(1);
   quadGenHeightSpinBox_->setMaximum(1000);
   quadGenHeightSpinBox_->setSingleStep(1);
   quadGenHeightSpinBox_->setValue(10);
   quadGenLayout->addWidget(quadGenHeightSpinBox_);
   mainToolboxLayout->addItem(quadGenLayout);

   // Triangle generation
   QHBoxLayout* triGenLayout = new QHBoxLayout(toolBox_);
   triGenLayout->setSpacing(5);
   QPushButton* triGenButton = new QPushButton("Generate triangle mesh", toolBox_);
   triGenLayout->addWidget(triGenButton);
   triGenRadiusSpinBox_ = new QSpinBox(toolBox_);
   triGenRadiusSpinBox_->setMinimum(0);
   triGenRadiusSpinBox_->setMaximum(20);
   triGenRadiusSpinBox_->setSingleStep(1);
   triGenRadiusSpinBox_->setValue(2);
   triGenLayout->addWidget(triGenRadiusSpinBox_);
   mainToolboxLayout->addItem(triGenLayout);

   ////////////////////////////////////////////////////////////////////////////////////////////////
   // Spacer at the end
   mainToolboxLayout->addItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Fixed));

   /// Connect signals->slots
   connect(selectButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(drawButton_, SIGNAL(clicked()), this, SLOT(slotSelectionButtonClicked()));
   connect(cutButton, SIGNAL(clicked()), this, SLOT(slotCutSelectedEdges()));
   connect(attachButton, SIGNAL(clicked()), this, SLOT(slotAttachSelected()));
   connect(splitVertexButton, SIGNAL(clicked()), this, SLOT(slotSplitVertex()));

   connect(useShapeToolsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(slotUseShapeToolsCheckBoxToggled()));
   connect(fixSelectedButton, SIGNAL(clicked()), this, SLOT(slotFixSelectedVertices()));
   connect(rigidifyButton, SIGNAL(clicked()), this, SLOT(slotRigidify()));
   connect(edgeStrainCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(edgeStrainWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(triangleStrainCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(triangleStrainWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(areaConstraintCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(areaConstraintMinSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(areaConstraintMaxSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(areaConstraintWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(bendingConstraintCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(bendingConstraintMinSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(bendingConstraintMaxSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(bendingConstraintWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(rectConstraintCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(rectConstraintWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(angleConstraintCheckBox_, SIGNAL(stateChanged(int)), this, SLOT(slotConstraintCheckBoxToggled()));
   connect(angleConstraintMinSpinBox_, SIGNAL(valueChanged(int)), this, SLOT(slotFlagUpdate()));
   connect(angleConstraintMaxSpinBox_, SIGNAL(valueChanged(int)), this, SLOT(slotFlagUpdate()));
   connect(angleConstraintWeightSpinBox_, SIGNAL(valueChanged(double)), this, SLOT(slotFlagUpdate()));
   connect(updateButton, SIGNAL(clicked()), this, SLOT(slotUpdateMesh()));

   connect(quadGenButton, SIGNAL(clicked()), this, SLOT(slotQuadGen()));
   connect(triGenButton, SIGNAL(clicked()), this, SLOT(slotTriGen()));
   /// End connect signals->slots

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
   } else if (object_updated_ && PluginFunctions::pickMode() == "MoveSelection" &&
              PluginFunctions::actionMode() == Viewer::PickingMode) {
      updateMesh();
      object_updated_ = false;
   }
}

/**
 * @brief MeshCut::slotObjectUpdated
 * @param _id The id of the updated object
 * @param _type
 */
void MeshCut::slotObjectUpdated(int _id, const UpdateType& _type) {
   if (use_shape_tools_ && _type == UPDATE_GEOMETRY) {
      object_updated_ = true;
   } else if (use_shape_tools_ && _id == shape_tools_->getObjId() && _type == UPDATE_TOPOLOGY) {
      shape_tools_->flagUpdateNeeded();
      object_updated_ = true;
   }
}

/**
 * @brief MeshCut::slotRestored
 * @param _objectid
 */
void MeshCut::slotRestored(int _objectid) {
   if (_objectid == shape_tools_->getObjId()) {
      shape_tools_->flagUpdateNeeded();
      object_updated_ = true;
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

//         // If real time cutting is selected, apply after two face-overs
//         if (directCutCheckBox_->isChecked() && cutting_tools_->faceOvers() > 2) {
//            /// TODO: real time cutting is hardcore
//            emit log(LOGWARN, "Real-time cutting is still under development.");
//         }
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

//   emit updatedObject(object->id(), UPDATE_COLOR);
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
   if (clampToEdgeCheckBox_->isChecked()) {
       emit log(LOGOUT, "Selected path on mesh");
       emit updatedObject(object->id(), UPDATE_SELECTION);
       emit updateView();
       emit createBackup(object->id(), "Path application", UPDATE_SELECTION);
   } else {
       emit log(LOGOUT, "Drew curve on mesh");
       emit updatedObject(object->id(), UPDATE_TOPOLOGY);
       emit updateView();
       emit createBackup(object->id(), "Path application", UPDATE_TOPOLOGY);
   }
}

/** \brief Update mesh after a change in topology or geometry
 *
 */
void MeshCut::updateMesh(bool _specify_n_iterations) {
   PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         // Find handle index
         std::vector<int> handle_idxs;
         TriMesh::VertexIter v_it(mesh.vertices_begin());
         for (; v_it!=mesh.vertices_end(); ++v_it) {
            if (mesh.status(*v_it).selected()) {
               handle_idxs.push_back((*v_it).idx());
            }
         }
         shape_tools_->setHandles(handle_idxs);
         shape_tools_->setWeightsAndRanges(edgeStrainWeightSpinBox_->value(),
                                           triangleStrainWeightSpinBox_->value(),
                                           areaConstraintMinSpinBox_->value(),
                                           areaConstraintMaxSpinBox_->value(),
                                           areaConstraintWeightSpinBox_->value(),
                                           bendingConstraintMinSpinBox_->value(),
                                           bendingConstraintMaxSpinBox_->value(),
                                           bendingConstraintWeightSpinBox_->value(),
                                           rectConstraintWeightSpinBox_->value(),
                                           angleConstraintMinSpinBox_->value()*DEG2RAD,
                                           angleConstraintMaxSpinBox_->value()*DEG2RAD,
                                           angleConstraintWeightSpinBox_->value());

         if (shape_tools_->updateNeeded()) {
            shape_tools_->setMesh(&mesh, o_it->id());

            emit log(LOGOUT, "TriMesh set for shape tools");
            emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
            emit createBackup(o_it->id(), "ShapeTools: Mesh set", UPDATE_GEOMETRY);
         }
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         // Find handle index
         std::vector<int> handle_idxs;
         PolyMesh::VertexIter v_it(mesh.vertices_begin());
         for (; v_it!=mesh.vertices_end(); ++v_it) {
            if (mesh.status(*v_it).selected()) {
               handle_idxs.push_back((*v_it).idx());
            }
         }
         shape_tools_->setHandles(handle_idxs);
         shape_tools_->setWeightsAndRanges(edgeStrainWeightSpinBox_->value(),
                                           triangleStrainWeightSpinBox_->value(),
                                           areaConstraintMinSpinBox_->value(),
                                           areaConstraintMaxSpinBox_->value(),
                                           areaConstraintWeightSpinBox_->value(),
                                           bendingConstraintMinSpinBox_->value(),
                                           bendingConstraintMaxSpinBox_->value(),
                                           bendingConstraintWeightSpinBox_->value(),
                                           rectConstraintWeightSpinBox_->value(),
                                           angleConstraintMinSpinBox_->value()*DEG2RAD,
                                           angleConstraintMaxSpinBox_->value()*DEG2RAD,
                                           angleConstraintWeightSpinBox_->value());

         if (shape_tools_->updateNeeded()) {
            shape_tools_->setMesh(&mesh, o_it->id());

            emit log(LOGOUT, "PolyMesh set for shape tools");
            emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
            emit createBackup(o_it->id(), "ShapeTools: Mesh set", UPDATE_GEOMETRY);
         }
      }

      if (_specify_n_iterations) {
         shape_tools_->solveUpdateMesh(nSolverIterationsSpinBox_->value());
      } else {
         shape_tools_->solveUpdateMesh();
      }

      emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
      emit updateView();

      /// TODO: support multiple objects
      break;
   }
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
            if (mesh.status(*e_it).selected() && cutting_tools_->cutPrimitive(*e_it, mesh, !independantCutsCheckBox_->isChecked())) {
               mesh.status(*e_it).set_selected(false);
               // Cut and count
               ++n_cuts;
            }
         }

         mesh.update_normals();

         emit log(LOGOUT, "Cut " + QString::number(n_cuts) + " TriMesh edges");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected edges cut", UPDATE_TOPOLOGY);
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         // Iterate over all edges
         size_t n_cuts(0);
         PolyMesh::EdgeIter e_it, e_end(mesh.edges_end());
         for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
            if (mesh.status(*e_it).selected() && cutting_tools_->cutPrimitive(*e_it, mesh, !independantCutsCheckBox_->isChecked())) {
               mesh.status(*e_it).set_selected(false);
               ++n_cuts;
            }
         }

         mesh.update_normals();

         emit log(LOGOUT, "Cut " + QString::number(n_cuts) + " PolyMesh edges");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected edges cut", UPDATE_TOPOLOGY);
      }
   }
}

/** \brief Attach selected vertices together
 *
 */
void MeshCut::slotAttachSelected() {
   PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         /// TODO: find closest pairs
         // Get both vertices
         bool found = false;
         TriMesh::VertexHandle vh0, vh1;
         TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               if (!found) {
                  vh0 = *v_it;
                  found = true;
               } else {
                  vh1 = *v_it;
                  found = false;
                  break;
               }
            }
         }

         // And attach
         if (!found && vh0 != vh1) {
            cutting_tools_->attachVertices(vh0, vh1, mesh);
         }
         mesh.status(vh0).set_selected(false);
         mesh.status(vh1).set_selected(false);

         mesh.update_normals();

         emit log(LOGOUT, "Attached TriMesh vertices");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Attached vertices", UPDATE_TOPOLOGY);
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         // Get both vertices
         bool found = false;
         PolyMesh::VertexHandle vh0, vh1;
         PolyMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               if (!found) {
                  vh0 = *v_it;
                  found = true;
               } else {
                  vh1 = *v_it;
                  found = false;
                  break;
               }
            }
         }

         // And attach
         if (!found && vh0.idx() != vh1.idx()) {
            cutting_tools_->attachVertices(vh0, vh1, mesh);
         }
         mesh.status(vh0).set_selected(false);
         mesh.status(vh1).set_selected(false);

         mesh.update_normals();

         emit log(LOGOUT, "Attached Polymesh vertices");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Attached vertices", UPDATE_TOPOLOGY);
      }
   }
}

/** \brief Split all selected vertices
 *
 * Every selected vertex is split if it is adjacent to two cuts.
 */
void MeshCut::slotSplitVertex() {
   PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS);
   for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
      if (o_it->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

         size_t n_splits(0);
         TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it!=v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               mesh.status(*v_it).set_selected(false);
               cutting_tools_->splitVertex(*v_it, mesh);
               ++n_splits;
            }
         }

         mesh.update_normals();

         emit log(LOGOUT, "Split " + QString::number(n_splits) + " TriMesh vertices");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected vertices split", UPDATE_TOPOLOGY);
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         size_t n_splits(0);
         PolyMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it!=v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               mesh.status(*v_it).set_selected(false);
               cutting_tools_->splitVertex(*v_it, mesh);
               ++n_splits;
            }
         }

         mesh.update_normals();

         emit log(LOGOUT, "Split " + QString::number(n_splits) + " PolyMesh vertices");
         emit updatedObject(o_it->id(), UPDATE_TOPOLOGY);
         emit updateView();
         emit createBackup(o_it->id(), "Selected vertices split", UPDATE_TOPOLOGY);
      }
   }
}

/** \brief Toggle the use of shape tools
 *
 */
void MeshCut::slotUseShapeToolsCheckBoxToggled() {
   use_shape_tools_ = !use_shape_tools_;
   shape_tools_->flagUpdateNeeded();
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

         TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               v_idxs.insert((*v_it).idx());

               mesh.status(*v_it).set_selected(false);
               /// TODO: find a way to color that
//               mesh.set_color(*v_it, OpenMesh::Vec4f(0.4f,0.2f,0.6f,1.0f));
            }
         }
      } else if (o_it->dataType(DATA_POLY_MESH)) {
         PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

         PolyMesh::VertexIter v_it, v_end(mesh.vertices_end());
         for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
            if (mesh.status(*v_it).selected()) {
               v_idxs.insert((*v_it).idx());

               mesh.status(*v_it).set_selected(false);
               /// TODO: find a way to color that
//               mesh.set_color(*v_it, OpenMesh::Vec4f(0.4f,0.2f,0.6f,1.0f));
            }
         }
      }

      shape_tools_->fixVertices(v_idxs);
      emit updatedObject(o_it->id(), UPDATE_SELECTION);

      /// At the moment, only one object is supported
      break;
   }

   emit updateView();
}

/** \brief Sets a set of vertices to be rigid with respect to each other
 *
 */
void MeshCut::slotRigidify() {
    std::vector<int> v_idxs;
    PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS);
    for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
       v_idxs.clear();

       if (o_it->dataType(DATA_TRIANGLE_MESH)) {
          TriMesh& mesh = *PluginFunctions::triMesh(*o_it);

          TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
          for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
             if (mesh.status(*v_it).selected()) {
                v_idxs.push_back((*v_it).idx());

                mesh.status(*v_it).set_selected(false);
             }
          }
       } else if (o_it->dataType(DATA_POLY_MESH)) {
          PolyMesh& mesh = *PluginFunctions::polyMesh(*o_it);

          PolyMesh::VertexIter v_it, v_end(mesh.vertices_end());
          for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
             if (mesh.status(*v_it).selected()) {
                v_idxs.push_back((*v_it).idx());

                mesh.status(*v_it).set_selected(false);
             }
          }
       }

       shape_tools_->setRigid(v_idxs);
       emit updatedObject(o_it->id(), UPDATE_SELECTION);

       /// At the moment, only one object is supported
       break;
    }

    emit updateView();
}

/** \brief Flag update to shape tools
 *
 */
void MeshCut::slotFlagUpdate() {
   shape_tools_->flagUpdateNeeded();
}

/** \brief Inform the shape tools of a checkbox action
 *
 */
void MeshCut::slotConstraintCheckBoxToggled() {
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::EDGE_STRAIN, edgeStrainCheckBox_->isChecked());
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::TRIANGLE_STRAIN, triangleStrainCheckBox_->isChecked());
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::AREA, areaConstraintCheckBox_->isChecked());
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::BENDING, bendingConstraintCheckBox_->isChecked());
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::RECT, rectConstraintCheckBox_->isChecked());
   shape_tools_->toggleConstraint(ShapeTools::ConstraintType::ANGLE, angleConstraintCheckBox_->isChecked());
   shape_tools_->flagUpdateNeeded();
}

/** \brief Trigger a mesh update
 *
 */
void MeshCut::slotUpdateMesh() {
   updateMesh(true);
}

/** \brief Generate a new quad mesh and add it to the scene
 *
 */
void MeshCut::slotQuadGen() {
   int id = -1;
   emit addEmptyObject(DATA_POLY_MESH, id);

   PolyMeshObject* meshObject = 0;
   PluginFunctions::getObject(id, meshObject);

   if (meshObject) {
      mesh_generator_->generateQuadRect(meshObject->mesh(), quadGenWidthSpinBox_->value(),
                                        quadGenHeightSpinBox_->value(), hingedTessellationCheckBox_->isChecked());

      emit log(LOGOUT, "Created quad mesh");
      emit updatedObject(id, UPDATE_TOPOLOGY);
      emit updateView();
      emit createBackup(id, "Mesh create", UPDATE_TOPOLOGY);
   }
}

/** \brief Generate a new triangle mesh and add it to the scene
 *
 */
void MeshCut::slotTriGen() {
   int id = -1;
   emit addEmptyObject(DATA_TRIANGLE_MESH, id);

   TriMeshObject* meshObject = 0;
   PluginFunctions::getObject(id, meshObject);

   if (meshObject) {
      mesh_generator_->generateTriHex(meshObject->mesh(), triGenRadiusSpinBox_->value(),
                                       hingedTessellationCheckBox_->isChecked());

      emit log(LOGOUT, "Created triangle mesh");
      emit updatedObject(id, UPDATE_TOPOLOGY);
      emit updateView();
      emit createBackup(id, "Mesh create", UPDATE_TOPOLOGY);
   }
}













