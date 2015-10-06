#include "MeshCut.hh"
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#define EDGE_CUT_POPUP "<B>Edge Cut</B><br>Cut along selected edge"

Q_EXPORT_PLUGIN2( meshCut , MeshCut );

MeshCut::MeshCut() : toolBar_(0), edgeCutAction_(0){}

/** \brief initialize the plugin
 *
 */
void MeshCut::pluginsInitialized()
{
   QString iconPath = OpenFlipper::Options::iconDirStr()+OpenFlipper::Options::dirSeparator();


   // // Cutting toolbox
   // QWidget* toolBox = new QWidget();

   // QPushButton* cutButton = new QPushButton("&Cut", toolBox);

   // QGridLayout* layout = new QGridLayout(toolBox);
   // layout->addWidget(cutButton, 0, 0);

   // connect(cutButton, SIGNAL(clicked()), this, SLOT(simpleCut()));

   // emit addToolbox(tr("Mesh Cut"), toolBox);


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
}

/** \brief Toolbar action trigger
 * @param _action the triggered action
 */
 void MeshCut::toolBarTriggered(QAction* _action) {
   if (_action->text() == EDGE_CUT_POPUP)
      PluginFunctions::pickMode(EDGE_CUT_POPUP);

   PluginFunctions::actionMode(Viewer::PickingMode);
 }

 /** \brief Toggle actions when the PickMode changes
  * @param _mode the new PickMode
  */
void MeshCut::slotPickModeChanged(const std::string& _mode) {
   edgeCutAction_->setChecked(_mode == EDGE_CUT_POPUP);
}

/** \brief Called when mouse is clicked
 *
 */
void MeshCut::slotMouseEvent(QMouseEvent* _event) {
   if (_event->buttons() == Qt::RightButton)
      return;

   if (PluginFunctions::pickMode() == EDGE_CUT_POPUP) {
      edgeCut(_event);
   }
}

/** \brief Find the selected edge and introduce a cut
 * @param _event the mouse click
 */
void MeshCut::edgeCut(QMouseEvent* _event) {
   if (_event->type() != QEvent::MouseButtonPress)
      return;

   unsigned int node_idx, target_idx;
   ACG::Vec3d hit_point;
   if (PluginFunctions::scenegraphPick(ACG::SceneGraph::PICK_FACE, _event->pos(), node_idx, target_idx, &hit_point)) {
      BaseObjectData* object;
      if (PluginFunctions::getPickedObject(node_idx, object)) {
         // In the case of a triangle mesh
         if (object->picked(node_idx) && object->dataType(DATA_TRIANGLE_MESH)) {
            TriMesh& mesh = *PluginFunctions::triMesh(object);
            TriMesh::FaceHandle faceh = mesh.face_handle(target_idx);

            // Get all three halfedges on the clicked face
            TriMesh::FaceEdgeIter fedge_it(mesh, faceh);
            TriMesh::HalfedgeHandle he1 = mesh.halfedge_handle(*fedge_it, 0);
            ++fedge_it;
            TriMesh::HalfedgeHandle he2 = mesh.halfedge_handle(*fedge_it, 0);
            ++fedge_it;
            TriMesh::HalfedgeHandle he3 = mesh.halfedge_handle(*fedge_it, 0);

            // Find closest halfedge
            double min_dist = ACG::Geometry::distPointLineSquared(hit_point, mesh.point(mesh.to_vertex_handle(he1)), mesh.point(mesh.from_vertex_handle(he1)));
            TriMesh::EdgeHandle edge = mesh.edge_handle(he1);
            double dist = ACG::Geometry::distPointLineSquared(hit_point, mesh.point(mesh.to_vertex_handle(he2)), mesh.point(mesh.from_vertex_handle(he2)));
            if (dist < min_dist) {
               min_dist = dist;
               edge = mesh.edge_handle(he2);
            }
            dist = ACG::Geometry::distPointLineSquared(hit_point, mesh.point(mesh.to_vertex_handle(he3)), mesh.point(mesh.from_vertex_handle(he3)));
            if (dist < min_dist) {
               min_dist = dist;
               edge = mesh.edge_handle(he3);
            }

            // Cut edge
            cutPrimitive(edge, mesh);

            emit log(LOGOUT, "Picked edge " + QString::number(edge.idx()));
            emit updatedObject(object->id(), UPDATE_TOPOLOGY);
            emit updateView();
         }
         // In the case of a polymesh
         else if (object->picked(node_idx) && object->dataType(DATA_POLY_MESH)) {
            PolyMesh& mesh = *PluginFunctions::polyMesh(object);
            PolyMesh::FaceHandle faceh = mesh.face_handle(target_idx);

            // Get all halfedges on the face
            PolyMesh::FaceHalfedgeIter fhedge_it(mesh, faceh);
            std::vector<PolyMesh::HalfedgeHandle> halfEdgeHandles;
            for (;fhedge_it.is_valid(); ++fhedge_it) {
               halfEdgeHandles.push_back(*fhedge_it);
            }

            // Find nearest edge
            PolyMesh::EdgeHandle edge = mesh.edge_handle(*halfEdgeHandles.begin());
            double min_dist = ACG::Geometry::distPointLineSquared(hit_point, mesh.point(mesh.to_vertex_handle(*halfEdgeHandles.begin())), mesh.point(mesh.from_vertex_handle(*halfEdgeHandles.begin())));
            for (std::vector<PolyMesh::HalfedgeHandle>::iterator iter = halfEdgeHandles.begin(); iter != halfEdgeHandles.end(); ++iter) {
               double dist = ACG::Geometry::distPointLineSquared(hit_point, mesh.point(mesh.to_vertex_handle(*iter)), mesh.point(mesh.from_vertex_handle(*iter)));
               if (dist < min_dist) {
                  edge = mesh.edge_handle(*iter);
                  min_dist = dist;
               }
            }

            // Cut edge
            // cutPrimitive(edge, mesh);

            emit log(LOGOUT, "Picked edge " + QString::number(edge.idx()));
            emit updatedObject(object->id(), UPDATE_TOPOLOGY);
            emit updateView();
         }
      }
   }
}

/** \brief Look at the surrounding topology of the edge and cut along it
 *
 */
void MeshCut::cutPrimitive(TriMesh::EdgeHandle _edge, TriMesh& _mesh) {

}

// /** \brief
//  * 
//  */
// void MeshCut::simpleCut() {

// }













