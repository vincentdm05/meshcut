#include "ShapeTools.hh"

ShapeTools::ShapeTools () : object_id_(-1), triMesh_(0),
   fixedVerticesIdx_(), fixedConstraintIds_() {
   solver_ = new ShapeOp::Solver();
}

void ShapeTools::updatePositions() {

}

void ShapeTools::setMesh(TriMesh *_mesh, int _object_id) {
   object_id_ = _object_id;
   triMesh_ = _mesh;

   ShapeOp::Matrix3X p(3, triMesh_->n_vertices());
   for (TriMesh::VertexIter v_it = triMesh_->vertices_begin(); v_it != triMesh_->vertices_end(); ++v_it) {
      int id = (*v_it).idx();

      Eigen::Vector3f pos(0, 0, 0);
      pos[0] = triMesh_->point(*v_it)[0];
      pos[1] = triMesh_->point(*v_it)[1];
      pos[2] = triMesh_->point(*v_it)[2];

      p.col(id) = pos.cast<ShapeOp::Scalar>();
   }

   solver_->setPoints(p);
}

/**
 * @brief ShapeTools::fixVertices
 * @param v_idxs Indices of vertices to fix
 */
void ShapeTools::toggleFixVertices(std::set<int> v_idxs) {
   if (v_idxs == fixedVerticesIdx_) {
      fixedVerticesIdx_.clear();
      return;
   }

   std::set<int>::iterator fixed_v_it(v_idxs.begin());
   for (; fixed_v_it!=v_idxs.end(); ++fixed_v_it) {
      int idx = *fixed_v_it;
      std::set<int>::iterator found_it = fixedVerticesIdx_.find(idx);
      if (found_it == v_idxs.end()) {
         fixedVerticesIdx_.insert(idx);
      } else {
         fixedVerticesIdx_.erase(found_it);
      }
   }
}

/**
 * @brief Update positions of vertices with respect to constraints
 */
bool ShapeTools::updateSolveMesh() {
   // Update vertex positions
   updatePositions();

   return true;
}











