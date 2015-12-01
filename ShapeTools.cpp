#include "ShapeTools.hh"

/**
 * @brief ShapeTools::setConstraints Sets all constraints, provided that indices of the relevant
 * elements are set appropriately.
 */
void ShapeTools::setConstraints() {
   // Closeness constraints for fixed vertices
   std::set<int>::iterator fixed_v_idxs_it(fixedVerticesIdx_.begin());
   for (; fixed_v_idxs_it!=fixedVerticesIdx_.end(); ++fixed_v_idxs_it) {
      std::vector<int> idx;
      idx.push_back(*fixed_v_idxs_it);

      // Add closeness constraint for fixed vertex
      auto c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", idx, fixedConstraintWeight_, solver_->getPoints());
      if (c) solver_->addConstraint(c);
   }

   // Closeness constraints for handles
   handleConstraintIds_.clear();
   std::vector<int>::iterator idx_it(handleIdxs_.begin());
   for (; idx_it!=handleIdxs_.end(); ++idx_it) {
      std::vector<int> idx;
      idx.push_back(*idx_it);

      auto c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", idx, handleConstraintWeight_, solver_->getPoints());
      if (c) handleConstraintIds_.insert(std::make_pair(*idx_it, solver_->addConstraint(c)));
   }

   // Edge strain
   if (edgeStrainActive_) {
      if (triMesh_) {
         TriMesh::EdgeIter e_it(triMesh_->edges_begin());
         std::vector<int> edge_v_idxs;
         for (; e_it!=triMesh_->edges_end(); ++e_it) {
            edge_v_idxs.clear();

            edge_v_idxs.push_back((triMesh_->to_vertex_handle(triMesh_->halfedge_handle(*e_it, 0))).idx());
            edge_v_idxs.push_back((triMesh_->to_vertex_handle(triMesh_->halfedge_handle(*e_it, 1))).idx());

            auto c = ShapeOp::Constraint::shapeConstraintFactory("EdgeStrain", edge_v_idxs, edgeStrainWeight_, solver_->getPoints());
            if (c) solver_->addConstraint(c);
         }
      } else {
         TriMesh::EdgeIter e_it(polyMesh_->edges_begin());
         std::vector<int> edge_v_idxs;
         for (; e_it!=polyMesh_->edges_end(); ++e_it) {
            edge_v_idxs.clear();

            edge_v_idxs.push_back((polyMesh_->to_vertex_handle(polyMesh_->halfedge_handle(*e_it, 0))).idx());
            edge_v_idxs.push_back((polyMesh_->to_vertex_handle(polyMesh_->halfedge_handle(*e_it, 1))).idx());

            auto c = ShapeOp::Constraint::shapeConstraintFactory("EdgeStrain", edge_v_idxs, edgeStrainWeight_, solver_->getPoints());
            if (c) solver_->addConstraint(c);
         }
      }
   }


   // Triangle constraint
   if (triangleStrainActive_ && triMesh_) {
      std::vector<int> v_ids;
      for (TriMesh::FaceIter f_it = triMesh_->faces_begin(); f_it != triMesh_->faces_end(); ++f_it)
      {
         v_ids.clear();
         for (TriMesh::FaceVertexIter fv_it = triMesh_->fv_iter(*f_it); fv_it; ++fv_it)
            v_ids.push_back((*fv_it).idx());

         auto c = ShapeOp::Constraint::shapeConstraintFactory("TriangleStrain", v_ids, triangleStrainWeight_, solver_->getPoints());
         if (c) solver_->addConstraint(c);
      }
   }

   // Area constraint
   if (areaConstraintActive_) {
      if (triMesh_) {
         std::vector<int> v_ids;
         for (TriMesh::FaceIter f_it = triMesh_->faces_begin(); f_it != triMesh_->faces_end(); ++f_it)
         {
            v_ids.clear();
            for (TriMesh::FaceVertexIter fv_it = triMesh_->fv_iter(*f_it); fv_it; ++fv_it)
               v_ids.push_back((*fv_it).idx());

            auto c = std::make_shared<ShapeOp::AreaConstraint>(v_ids, areaConstraintWeight_, solver_->getPoints());
            if (c) {
               c->setRangeMin(areaMin_);
               c->setRangeMax(areaMax_);
               solver_->addConstraint(c);
            }
         }
      } else {
         std::vector<int> v_ids;
         for (PolyMesh::FaceIter f_it = polyMesh_->faces_begin(); f_it != polyMesh_->faces_end(); ++f_it)
         {
            v_ids.clear();
            size_t n_vertices = 0;
            PolyMesh::FaceVertexIter fv_it = polyMesh_->fv_iter(*f_it);
            int first_v_idx = (*fv_it).idx();
            for (; fv_it; ++fv_it) {
               v_ids.push_back((*fv_it).idx());
               ++n_vertices;

               if (n_vertices == 3) {
                  n_vertices = 1;

                  auto c = std::make_shared<ShapeOp::AreaConstraint>(v_ids, areaConstraintWeight_, solver_->getPoints());
                  if (c) {
                     c->setRangeMin(areaMin_);
                     c->setRangeMax(areaMax_);
                     solver_->addConstraint(c);
                  }

                  v_ids.clear();
                  v_ids.push_back((*fv_it).idx());
               }
            }

            if (n_vertices == 2) {
               v_ids.push_back(first_v_idx);
               auto c = std::make_shared<ShapeOp::AreaConstraint>(v_ids, areaConstraintWeight_, solver_->getPoints());
               if (c) {
                  c->setRangeMin(areaMin_);
                  c->setRangeMax(areaMax_);
                  solver_->addConstraint(c);
               }
            }
         }
      }
   }

   // Bending constraint
   if (bendingConstraintActive_ && triMesh_) {
      std::vector<int> v_ids;
      for (TriMesh::EdgeIter e_it = triMesh_->edges_begin(); e_it != triMesh_->edges_end(); ++e_it) {
         v_ids.clear();

         if (!triMesh_->is_boundary(*e_it)) {
            TriMesh::HalfedgeHandle heh = triMesh_->halfedge_handle(*e_it, 0);
            v_ids.push_back(triMesh_->from_vertex_handle(heh).idx());
            v_ids.push_back(triMesh_->to_vertex_handle(heh).idx());
            v_ids.push_back(triMesh_->from_vertex_handle(triMesh_->prev_halfedge_handle(heh)).idx());
            v_ids.push_back(triMesh_->to_vertex_handle(triMesh_->next_halfedge_handle(triMesh_->opposite_halfedge_handle(heh))).idx());

            auto c = std::make_shared<ShapeOp::BendingConstraint>(v_ids, bendingConstraintWeight_, solver_->getPoints(), bendingMin_, bendingMax_);
            if (c) solver_->addConstraint(c);
         }
      }
   }

   // Rectangle constraint
   if (rectConstraintActive_ && polyMesh_) {
      std::vector<int> v_ids;
      for (PolyMesh::FaceIter f_it = polyMesh_->faces_begin(); f_it != polyMesh_->faces_end(); ++f_it) {
         v_ids.clear();
         if (polyMesh_->valence(*f_it) == 4) {
            for (PolyMesh::FaceVertexIter fv_it = polyMesh_->fv_iter(*f_it); fv_it; ++fv_it) {
               v_ids.push_back(fv_it->idx());
            }

            auto c = std::make_shared<ShapeOp::RectangleConstraint>(v_ids, rectConstraintWeight_, solver_->getPoints());
            if (c) solver_->addConstraint(c);
         }
      }
   }

}

/**
 * @brief ShapeTools::setMesh Saves a pointer to the mesh, initialises a solver and sets the positions
 * of vertices in the solver.
 * This is the trimesh version and sets the polymesh pointer to 0.
 *
 * @param _mesh The mesh pointer
 * @param _object_id The id of the object as set by openflipper, for later reference
 */
void ShapeTools::setMesh(TriMesh *_mesh, int _object_id) {
   object_id_ = _object_id;
   triMesh_ = _mesh;
   polyMesh_ = 0;

   if (solver_ != NULL) delete solver_;
   solver_ = new ShapeOp::Solver();

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
 * @brief ShapeTools::setMesh Saves a pointer to the mesh, initialises a solver and sets the positions
 * of vertices in the solver.
 * This is the polymesh version and sets the trimesh pointer to 0.
 *
 * @param _mesh The mesh pointer
 * @param _object_id The id of the object as set by openflipper, for later reference
 */
void ShapeTools::setMesh(PolyMesh *_mesh, int _object_id) {
   object_id_ = _object_id;
   polyMesh_ = _mesh;
   triMesh_ = 0;

   if (solver_ != NULL) delete solver_;
   solver_ = new ShapeOp::Solver();

   // Set vertices positions in solver
   ShapeOp::Matrix3X p(3, polyMesh_->n_vertices());
   for (TriMesh::VertexIter v_it = polyMesh_->vertices_begin(); v_it != polyMesh_->vertices_end(); ++v_it) {
      TriMesh::Point point = polyMesh_->point(*v_it);

      Eigen::Vector3f pos(0, 0, 0);
      pos[0] = point[0];
      pos[1] = point[1];
      pos[2] = point[2];

      p.col((*v_it).idx()) = pos.cast<ShapeOp::Scalar>();
   }

   solver_->setPoints(p);
}

/**
 * @brief ShapeTools::moveHandle Updates the position of the handles in the solver
 */
void ShapeTools::moveHandles() {
   std::vector<int>::iterator idx_it(handleIdxs_.begin());
   for (; idx_it!=handleIdxs_.end(); ++idx_it) {

      if (triMesh_) {
         TriMesh::Point handle_point = triMesh_->point(triMesh_->vertex_handle(*idx_it));
         Eigen::Vector3f pos(handle_point[0], handle_point[1], handle_point[2]);
         auto c = std::dynamic_pointer_cast<ShapeOp::ClosenessConstraint>(solver_->getConstraint(handleConstraintIds_[*idx_it]));
         c->setPosition(pos.cast<ShapeOp::Scalar>());
      } else {
         PolyMesh::Point handle_point = polyMesh_->point(polyMesh_->vertex_handle(*idx_it));
         Eigen::Vector3f pos(handle_point[0], handle_point[1], handle_point[2]);
         auto c = std::dynamic_pointer_cast<ShapeOp::ClosenessConstraint>(solver_->getConstraint(handleConstraintIds_[*idx_it]));
         c->setPosition(pos.cast<ShapeOp::Scalar>());
      }
   }
}

bool ShapeTools::solveUpdateMesh() {
   return solveUpdateMesh(solver_iterations_);
}

/**
 * @brief Solve linear system and set the new positions of vertices
 */
bool ShapeTools::solveUpdateMesh(size_t _n_iterations) {
   if (!triMesh_ && !polyMesh_) {
      std::cerr << "A mesh should be set before using the solver" << std::endl;
      return false;
   }

   // Topology or handles have changed
   if (update_needed_) {
      setConstraints();
      solver_->initialize();

      update_needed_ = false;
   }

   // Update solver handle positions
   moveHandles();

   // Solve system
   if (!solver_->solve(_n_iterations)) return false;

   // Update vertex positions on mesh
   if (triMesh_) {
      ShapeOp::Matrix3X p(3, triMesh_->n_vertices());
      p = solver_->getPoints();
      TriMesh::VertexIter v_it(triMesh_->vertices_begin());
      for (; v_it!=triMesh_->vertices_end(); ++v_it) {
         Eigen::Vector3d pos = p.col((*v_it).idx());

         TriMesh::Point point(pos[0], pos[1], pos[2]);
         triMesh_->set_point(*v_it, point);
      }
      triMesh_->update_normals();
   } else {
      ShapeOp::Matrix3X p(3, polyMesh_->n_vertices());
      p = solver_->getPoints();
      TriMesh::VertexIter v_it(polyMesh_->vertices_begin());
      for (; v_it!=polyMesh_->vertices_end(); ++v_it) {
         Eigen::Vector3d pos = p.col((*v_it).idx());

         TriMesh::Point point(pos[0], pos[1], pos[2]);
         polyMesh_->set_point(*v_it, point);
      }
      polyMesh_->update_normals();
   }

   return true;
}










