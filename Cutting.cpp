#include "Cutting.hh"


/** \brief Follow path and mark edges for split
 * Edges crossed by the mouse path are stored with the crossing point for later splitting.
 */
void Cutting::markForSplit(BaseObjectData* object) {
   if (recorded_path_.size() < 2 || !object) return;

   // This is only a hack to prevent oscillating and getting into an infinite loop
   std::pair<int,int> infinite_loop_breaker[3];

   while(recorded_path_.size() > 1) {
      // Advance until there is a change of face
      PathPoint curr_point = recorded_path_.front();
      recorded_path_.pop_front();
      PathPoint next_point = recorded_path_.front();
      while (recorded_path_.size() > 1 && std::get<TUPLE_FACE>(next_point) == std::get<TUPLE_FACE>(curr_point)) {
         curr_point = next_point;
         recorded_path_.pop_front();
         next_point = recorded_path_.front();
      }
      if (recorded_path_.size() < 2 && std::get<TUPLE_FACE>(next_point) == std::get<TUPLE_FACE>(curr_point)) break;

      /// TODO: Find and correct bug so that we don't need that
      /// It is a terrible hack and we should not have to rely on this. ///
      infinite_loop_breaker[0] = infinite_loop_breaker[1];
      infinite_loop_breaker[1] = infinite_loop_breaker[2];
      infinite_loop_breaker[2] = std::make_pair(std::get<TUPLE_FACE>(curr_point), std::get<TUPLE_FACE>(next_point));
//      std::cout << std::get<TUPLE_FACE>(curr_point) << " " << std::get<TUPLE_FACE>(next_point) << std::endl;
      if ((infinite_loop_breaker[2].second == infinite_loop_breaker[1].second &&
           infinite_loop_breaker[2].first == infinite_loop_breaker[1].first) ||
          (infinite_loop_breaker[2].second == infinite_loop_breaker[0].second &&
           infinite_loop_breaker[2].first == infinite_loop_breaker[0].first)) {
         recorded_path_.pop_front();
         continue;
      }
      /// End of hack ///

      // Get the two points and their closest edge
      ACG::Vec3d curr_hit_point = std::get<TUPLE_POINT>(curr_point);
      int curr_edge = std::get<TUPLE_EDGE>(curr_point);
      ACG::Vec3d next_hit_point = std::get<TUPLE_POINT>(next_point);
      int next_edge = std::get<TUPLE_EDGE>(next_point);

      /// Triangle mesh
      if (object->dataType(DATA_TRIANGLE_MESH)) {
         TriMesh& mesh = *PluginFunctions::triMesh(object);

         /// Find crossing(s) and mark for split
         // Points are on both sides of the same edge
         if (curr_edge == next_edge) {
            TriMesh::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(next_edge), 0);

            // Vector from current point to next
            ACG::Vec3d curr_to_next = next_hit_point - curr_hit_point;

            // Distance from current point to edge
            double curr_point_edge_dist = ACG::Geometry::distPointLine(curr_hit_point,
                     mesh.point(mesh.from_vertex_handle(heh)),
                     mesh.point(mesh.to_vertex_handle(heh)));

            // Distance from next point to edge
            double next_point_edge_dist = ACG::Geometry::distPointLine(next_hit_point,
                     mesh.point(mesh.from_vertex_handle(heh)),
                     mesh.point(mesh.to_vertex_handle(heh)));

            // Crossing point distance ratio
            double ratio = curr_point_edge_dist / (curr_point_edge_dist + next_point_edge_dist);

            // Actual crossing point
            ACG::Vec3d edge_crossing(curr_hit_point + ratio * curr_to_next);

            // Store edge crossing
            edges_to_split_.push(std::make_pair(next_edge, edge_crossing));
         }
         // Points are further apart
         else {
            /**
             * p0 is the point inside the face, corresponding here to curr_hit_point
             * p1 is the projected point outside the face. It is projected onto the plane defined by the
             *    face on which p0 resides and together with p0 define a line we wish to intersect with
             *    one of the edges of the current face.
             */
            // Project end point to plane defined by triangle under first point
            ACG::Vec3d projected_end = projectToFacePlane(next_hit_point, std::get<TUPLE_FACE>(curr_point), mesh);

            // Find crossed edge and point
            int crossed_edge_idx = -1;
            /// TODO: idea: if crossing is close to vertex, use that insead.
            ACG::Vec3d intersect_p = findOutgoingFaceCrossing(curr_hit_point, projected_end,
                                                              std::get<TUPLE_FACE>(curr_point), crossed_edge_idx, mesh);
            Eigen::Vector3d intersection_point(intersect_p[0], intersect_p[1], intersect_p[2]);

            // Find next face
            int next_face_idx = -1;
            // No edge was crossed
            if (crossed_edge_idx == -1) {
               // First get vertex index of crossing
               int v_idx = closest_connected_vertex(std::get<TUPLE_VERTEX>(curr_point), intersection_point, mesh);

               /// Circulate around both faces and surrounding edges to find out if the curve goes
               /// through that face.
               TriMesh::VertexVertexIter vvit = mesh.vv_iter(mesh.vertex_handle(v_idx));
               TriMesh::VertexFaceIter vfit = mesh.vf_iter(mesh.vertex_handle(v_idx));
               TriMesh::VertexHandle vh0 = *vvit; ++vvit;
               Eigen::Vector3d end_point(next_hit_point[0], next_hit_point[1], next_hit_point[2]);
               for (; vvit && vfit; ++vvit) {
                  TriMesh::VertexHandle vh1 = *vvit;
                  Eigen::Vector3d proj_end = projectToFacePlane(end_point, (*vfit).idx(), mesh);

                  Eigen::Vector3d q0(mesh.point(vh0)[0], mesh.point(vh0)[1], mesh.point(vh0)[2]);
                  Eigen::Vector3d q1(mesh.point(vh1)[0], mesh.point(vh1)[1], mesh.point(vh1)[2]);

                  /// TODO: check if iszero comparisons are valid
                  // Check intersection with opposite edge
                  Eigen::Vector3d intersect_on_facing_edge;
                  int intersect_status = segmentIntersect(intersection_point, proj_end, q0, q1, &intersect_on_facing_edge);
                  if ((intersect_status == INTERSECT_OK || intersect_status == INTERSECT_COLLINEAR) &&
                      !(intersect_on_facing_edge - intersection_point).isZero(1e-5) &&
                      !((intersection_point - q0).isZero(1e-5) || (intersection_point - q1).isZero(1e-5))) {
                     next_face_idx = (*vfit).idx();
                     break;
                  }

                  // Go to next face
                  if (!mesh.is_boundary(vh0) && !mesh.is_boundary(vh1)) ++vfit;
                  vh0 = vh1;
               }
            }
            // An edge was crossed
            else {
               TriMesh::HalfedgeHandle crossed_halfedge = mesh.halfedge_handle(mesh.edge_handle(crossed_edge_idx), 0);
               if ((mesh.face_handle(crossed_halfedge).idx()) == std::get<TUPLE_FACE>(curr_point))
                  crossed_halfedge = mesh.opposite_halfedge_handle(crossed_halfedge);
               next_face_idx = (mesh.face_handle(crossed_halfedge)).idx();
            }
            if (next_face_idx == -1) {
               std::cout << "Could not find next face" << std::endl;
               break;
            }

            // Mark edge and crossing point
            ACG::Vec3d crossing(intersection_point[0], intersection_point[1], intersection_point[2]);
            edges_to_split_.push(std::make_pair(crossed_edge_idx, crossing));

            // Project rest of line to next face
            projected_end = projectToFacePlane(next_hit_point, next_face_idx, mesh);

            // Place new point inside the triangle
            ACG::Vec3d point_in_face;
            ACG::Vec3d face_centroid = mesh.calc_face_centroid(mesh.face_handle(next_face_idx));
            TriMesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(next_face_idx));
            // Intersect the line with segments going from face centroid to face vertices
            for (; fv_it; ++fv_it) {
               ACG::Vec3d edge_point = mesh.point(*fv_it);

               // As soon as an edge is crossed, stop: we only need one
               if (segmentIntersect(crossing, projected_end,
                                    face_centroid, edge_point, &point_in_face) == INTERSECT_OK) break;
            }

            // Add new point to recorded path with next face
            recorded_path_.push_front(std::make_tuple(point_in_face,
                  next_face_idx, crossed_edge_idx, closestVertexOnFace(crossing,std::get<TUPLE_FACE>(curr_point),mesh)));
         }
      }
      /// Polygonal mesh
      else if (object->dataType(DATA_POLY_MESH)) {
         std::cout << "Warning: no support for drawing curves on polygonal meshes." << std::endl;
         return;
      }
   }
}






