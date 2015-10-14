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
 */
void MeshCut::cutPrimitive(TriMesh::EdgeHandle edge, TriMesh& mesh) {
    // Already a hole there
    if (mesh.is_boundary(edge)) {
        return;
    }

    /// Get relevant neighbouring components
    // Original edge's halfedges
    TriMesh::HalfedgeHandle heh_in = mesh.halfedge_handle(edge, 0);
    TriMesh::HalfedgeHandle heh_out = mesh.halfedge_handle(edge, 1);

    // Left face halfedges
    TriMesh::HalfedgeHandle left_up_heh_to_new = mesh.next_halfedge_handle(heh_out);
    TriMesh::HalfedgeHandle left_down_heh_to_new = mesh.next_halfedge_handle(left_up_heh_to_new);

    // Face adjacent to new edge
    TriMesh::FaceHandle fh_left = mesh.face_handle(heh_out);

    // Edge endpoints
    TriMesh::VertexHandle vh_up = mesh.from_vertex_handle(heh_in);
    TriMesh::VertexHandle vh_down = mesh.to_vertex_handle(heh_in);

    emit log(LOGOUT, " vh_up " + QString::number(vh_up.idx()) + " pos " + QString::number(mesh.point(vh_up)[0]) + " " + QString::number(mesh.point(vh_up)[1]) + " " + QString::number(mesh.point(vh_up)[2]));


    /// Create new edge, copy all properties and adjust pointers
    // Outward new halfedge
    TriMesh::HalfedgeHandle new_heh_out = mesh.new_edge(vh_down, vh_up);
    mesh.copy_all_properties(heh_in, new_heh_out, true);
    mesh.set_vertex_handle(new_heh_out, vh_down);
    mesh.set_next_halfedge_handle(new_heh_out, heh_out);
    mesh.set_next_halfedge_handle(heh_out, new_heh_out);
    // no need to set_prev_halfedge_handle as it is done in set_next_halfedge_handle
    mesh.set_halfedge_handle(vh_up, new_heh_out);

    // Inward new halfedge
    TriMesh::HalfedgeHandle new_heh_in = mesh.opposite_halfedge_handle(new_heh_out);
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
}

/** \brief Joins adjacent cuts
 *
 * If two adjacent cuts are detected, the edge will be split and the properties and pointers
 * of connected components will be updated.
 *
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
 * @param vh The connecting vertex
 */
void MeshCut::connectCuts(TriMesh::VertexHandle vh, TriMesh& mesh) {
    const size_t _max_cuts = 2;
    // Halfedges at cuts
    TriMesh::HalfedgeHandle heh_cuts[2*_max_cuts];
    // Halfedges on the left side of the cut
    std::vector<TriMesh::HalfedgeHandle> left_halfedges;
    bool left_side = true;
    size_t cut = 0;

    // Count cuts and store halfedges according to side
    for (TriMesh::VertexIHalfedgeIter vih_it = mesh.vih_iter(vh); vih_it.is_valid(); ++vih_it) {
        TriMesh::HalfedgeHandle heh = *vih_it;
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
        TriMesh::VertexHandle new_vh = mesh.add_vertex(mesh.point(vh));
        mesh.copy_all_properties(vh, new_vh, true);

        /// Update pointers; the new vertex will pick up left edges
        // Left vertex boundary halfedges
        mesh.set_next_halfedge_handle(heh_cuts[0], heh_cuts[3]);
        mesh.set_halfedge_handle(new_vh, heh_cuts[3]);
        mesh.set_vertex_handle(heh_cuts[0], new_vh);
        // Other left halfedges
        if (!left_halfedges.empty()) {
            std::vector<TriMesh::HalfedgeHandle>::iterator heh_it = left_halfedges.begin();
            for (; heh_it != left_halfedges.end(); ++heh_it) {
                mesh.set_vertex_handle(*heh_it, new_vh);
            }
        }

        // Right vertex
        mesh.set_next_halfedge_handle(heh_cuts[2], heh_cuts[1]);
        mesh.set_halfedge_handle(vh, heh_cuts[1]);
        mesh.set_vertex_handle(heh_cuts[2], vh);
        // No need to rewire right halfedges

        emit log(LOGOUT, "Created new vertex " + QString::number(new_vh.idx()));
    }
    emit log(LOGOUT, "cuts " + QString::number(cut));

}














