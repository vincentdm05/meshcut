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

                emit log(LOGOUT, "Cut edge " + QString::number(edge.idx()));
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

/** \brief Cut along edge
 *
 * Look at the surrounding topology of the edge and cut along it.
 * This schematic shows how the middle edge relates to its
 * surroundings.
 *
 *              up vertex
 *        ----------x----------
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
 */
void MeshCut::cutPrimitive(TriMesh::EdgeHandle _edge, TriMesh& _mesh) {
    // Already a hole there
    if (_mesh.is_boundary(_edge)) {
        return;
    }

    /// Get relevant neighbouring components
    // Original edge's halfedges
    TriMesh::HalfedgeHandle heh_in = _mesh.halfedge_handle(_edge, 0);
    TriMesh::HalfedgeHandle heh_out = _mesh.halfedge_handle(_edge, 1);

    // Left face halfedges
    TriMesh::HalfedgeHandle left_up_heh_to_new = _mesh.next_halfedge_handle(heh_out);
    TriMesh::HalfedgeHandle left_down_heh_to_new = _mesh.next_halfedge_handle(left_up_heh_to_new);

    // Face adjacent to new edge
    TriMesh::FaceHandle fh_left = _mesh.face_handle(heh_out);

    // Edge endpoints
    TriMesh::VertexHandle vh_up = _mesh.from_vertex_handle(heh_in);
    TriMesh::VertexHandle vh_down = _mesh.to_vertex_handle(heh_in);

    /// Create new edge, copy all properties and adjust pointers
    // Outward new halfedge
    TriMesh::HalfedgeHandle new_heh_out = _mesh.new_edge(vh_down, vh_up);
    _mesh.copy_all_properties(heh_in, new_heh_out, true);
    _mesh.set_vertex_handle(new_heh_out, vh_down);
    _mesh.set_next_halfedge_handle(new_heh_out, heh_out);
    _mesh.set_next_halfedge_handle(heh_out, new_heh_out);
    // no need to set_prev_halfedge_handle as it is done in set_next_halfedge_handle
    _mesh.set_halfedge_handle(vh_up, new_heh_out);

    // Inward new halfedge
    TriMesh::HalfedgeHandle new_heh_in = _mesh.opposite_halfedge_handle(new_heh_out);
    _mesh.copy_all_properties(heh_out, new_heh_out, true);
    _mesh.set_vertex_handle(new_heh_in, vh_up);
    _mesh.set_face_handle(new_heh_in, fh_left);
    _mesh.set_next_halfedge_handle(new_heh_in, left_up_heh_to_new);
    _mesh.set_next_halfedge_handle(left_down_heh_to_new, new_heh_in);
    if (_mesh.halfedge_handle(fh_left) == heh_out) {
        _mesh.set_halfedge_handle(fh_left, new_heh_in);
    }
    _mesh.set_halfedge_handle(vh_down, heh_out);

    // Outward halfedges are now boundary
    _mesh.set_boundary(new_heh_out);
    _mesh.set_boundary(heh_out);

    //   if (_mesh.status(newEdge).selected()) {
    //       _mesh.status(newEdge).set_selected(false);
    //   } else {
    //       _mesh.status(newEdge).set_selected(true);
    //   }
    //   _mesh.status(vertexh1).set_selected(true);

    //   _mesh.status(faceh_left).set_selected(true);
    //   _mesh.status(faceh_right).set_selected(true);

}

// /** \brief
//  * 
//  */
// void MeshCut::simpleCut() {

// }













