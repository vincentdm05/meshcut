#include "ShapeTools.hh"

ShapeTools::ShapeTools () : is_running_(false), object_id_(-1), triMesh_(0),
   fixedConstraintWeight_(1000.0), fixedVerticesIdx_(), fixedConstraintIds_(),
   handleConstraintWeight_(100000000.0), handleIdxs_(), handleConstraintIds_(),
   edgeStrainWeight_(0.0), edgeStrainConstraintIds_() {
   solver_ = new ShapeOp::Solver();
   solver_iterations_ = 50;
}

void ShapeTools::setConstraints() {
   // Closeness constraints for fixed vertices
   fixedConstraintIds_.clear();
   std::set<int>::iterator fixed_v_idxs_it(fixedVerticesIdx_.begin());
   for (; fixed_v_idxs_it!=fixedVerticesIdx_.end(); ++fixed_v_idxs_it) {
      std::vector<int> idx;
      idx.push_back(*fixed_v_idxs_it);

      // Add closeness constraint for fixed vertex
      auto c = std::make_shared<ShapeOp::ClosenessConstraint>(idx, fixedConstraintWeight_, solver_->getPoints());
      fixedConstraintIds_.push_back(solver_->addConstraint(c));
   }

   // Closeness constraints for handles
   handleConstraintIds_.clear();
   std::vector<int>::iterator idx_it(handleIdxs_.begin());
   for (; idx_it!=handleIdxs_.end(); ++idx_it) {
      std::vector<int> idx;
      idx.push_back(*idx_it);

      auto c = std::make_shared<ShapeOp::ClosenessConstraint>(idx, handleConstraintWeight_, solver_->getPoints());
      handleConstraintIds_.insert(std::make_pair(*idx_it, solver_->addConstraint(c)));
   }

   // Edge strain
   edgeStrainConstraintIds_.clear();
   TriMesh::EdgeIter e_it(triMesh_->edges_begin());
   for (; e_it!=triMesh_->edges_end(); ++e_it) {
      std::vector<int> edge_v_idxs;
      edge_v_idxs.push_back((triMesh_->to_vertex_handle(triMesh_->halfedge_handle(*e_it, 0))).idx());
      edge_v_idxs.push_back((triMesh_->to_vertex_handle(triMesh_->halfedge_handle(*e_it, 1))).idx());

      auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(edge_v_idxs, edgeStrainWeight_, solver_->getPoints());
      edgeStrainConstraintIds_.push_back(solver_->addConstraint(c));
   }
}

/**
 * @brief ShapeTools::setMesh
 * @param _mesh
 * @param _object_id
 */
void ShapeTools::setMesh(TriMesh *_mesh, int _object_id) {
   if (!_mesh || _object_id < 0) return;
   object_id_ = _object_id;
   triMesh_ = _mesh;

   // Set vertices positions in solver
   ShapeOp::Matrix3X p(3, triMesh_->n_vertices());
   for (TriMesh::VertexIter v_it = triMesh_->vertices_begin(); v_it != triMesh_->vertices_end(); ++v_it) {
      TriMesh::Point point = triMesh_->point(*v_it);

      Eigen::Vector3f pos(0, 0, 0);
      pos[0] = point[0];
      pos[1] = point[1];
      pos[2] = point[2];

      p.col((*v_it).idx()) = pos.cast<ShapeOp::Scalar>();
   }

   solver_->setPoints(p);
}

/**
 * @brief ShapeTools::fixVertices
 * @param v_idxs Indices of vertices to fix
 */
void ShapeTools::toggleFixVertices(std::set<int> v_idxs) {
   if (v_idxs.empty()) {
      fixedConstraintIds_.clear();
      return;
   }

   // Toggle indices of fixed vertices
   std::set<int>::iterator v_idxs_it(v_idxs.begin());
   for (; v_idxs_it!=v_idxs.end(); ++v_idxs_it) {
      int idx = *v_idxs_it;
      std::set<int>::iterator found_it = fixedVerticesIdx_.find(idx);

      if (found_it == fixedVerticesIdx_.end()) {
         fixedVerticesIdx_.insert(idx);
         triMesh_->status(triMesh_->vertex_handle(idx)).set_selected(false);
         /// TODO: find a way to color that
         triMesh_->set_color(triMesh_->vertex_handle(idx), OpenMesh::Vec4f(0.4f,0.2f,0.6f,1.0f));
      } else {
         fixedVerticesIdx_.erase(idx);
         triMesh_->status(triMesh_->vertex_handle(idx)).set_selected(false);
         triMesh_->set_color(triMesh_->vertex_handle(idx), OpenMesh::Vec4f(0.5,0.5,0.5,1));
      }
   }
}

/**
 * @brief ShapeTools::moveHandle
 */
void ShapeTools::moveHandles() {
   if (handleIdxs_.empty()) return;

   std::vector<int>::iterator idx_it(handleIdxs_.begin());
   for (; idx_it!=handleIdxs_.end(); ++idx_it) {
      TriMesh::Point handle_point = triMesh_->point(triMesh_->vertex_handle(*idx_it));
      Eigen::Vector3d pos(handle_point[0], handle_point[1], handle_point[2]);

      auto c = std::dynamic_pointer_cast<ShapeOp::ClosenessConstraint>(solver_->getConstraint(handleConstraintIds_[*idx_it]));
      c->setPosition(pos.cast<ShapeOp::Scalar>());
   }
}

/**
 * @brief Update positions of vertices based on new positions obtained from solving system
 */
void ShapeTools::solveUpdateMesh() {
   if (!triMesh_) return;

   // Update solver
   setConstraints();
   solver_->initialize();
   moveHandles();

   // Solve system
   solver_->solve(solver_iterations_);

   // Update vertex positions on mesh
   ShapeOp::Matrix3X p(3, triMesh_->n_vertices());
   p = solver_->getPoints();
   TriMesh::VertexIter v_it(triMesh_->vertices_begin());
   for (; v_it!=triMesh_->vertices_end(); ++v_it) {
      Eigen::Vector3d pos = p.col((*v_it).idx());

      TriMesh::Point point(pos[0], pos[1], pos[2]);
      triMesh_->set_point(*v_it, point);
   }
   triMesh_->update_normals();
}











