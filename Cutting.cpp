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
      PathPoint prev_point = recorded_path_.front();
      recorded_path_.pop_front();
      PathPoint curr_point = recorded_path_.front();
      while (recorded_path_.size() > 1 && std::get<TUPLE_FACE>(curr_point) == std::get<TUPLE_FACE>(prev_point)) {
         prev_point = curr_point;
         recorded_path_.pop_front();
         curr_point = recorded_path_.front();
      }
      if (recorded_path_.size() < 2 && std::get<TUPLE_FACE>(curr_point) == std::get<TUPLE_FACE>(prev_point)) break;

      /// TODO: Find and correct bug so that we don't need that
      /// It is a terrible hack and we should not rely on this
      infinite_loop_breaker[0] = infinite_loop_breaker[1];
      infinite_loop_breaker[1] = infinite_loop_breaker[2];
      infinite_loop_breaker[2] = std::make_pair(std::get<TUPLE_FACE>(prev_point), std::get<TUPLE_FACE>(curr_point));
//      std::cout << std::get<TUPLE_FACE>(prev_point) << " " << std::get<TUPLE_FACE>(curr_point) << std::endl;
      if ((infinite_loop_breaker[2].second == infinite_loop_breaker[1].second &&
           infinite_loop_breaker[2].first == infinite_loop_breaker[1].first) ||
          (infinite_loop_breaker[2].second == infinite_loop_breaker[0].second &&
           infinite_loop_breaker[2].first == infinite_loop_breaker[0].first)) {
         recorded_path_.pop_front();
         continue;
      }
      /// End of hack ///

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
            int crossed_edge_idx = -1;
            Eigen::Vector3d intersection_point = findOutgoingFaceCrossing(p0, p1, std::get<TUPLE_FACE>(prev_point),
                                                                          crossed_edge_idx, mesh);

            // Find next face
            int next_face_idx = -1;
            if (crossed_edge_idx == -1) {
               // First get vertex index of crossing
               int v_idx = closest_connected_vertex(std::get<TUPLE_VERTEX>(prev_point), intersection_point, mesh);

               /// Circulate around both faces and surrounding edges to find out if the curve goes
               /// through that face.
               TriMesh::VertexVertexIter vvit = mesh.vv_iter(mesh.vertex_handle(v_idx));
               TriMesh::VertexFaceIter vfit = mesh.vf_iter(mesh.vertex_handle(v_idx));
               TriMesh::VertexHandle vh0 = *vvit; ++vvit;
               Eigen::Vector3d end_point(curr_hit_point[0], curr_hit_point[1], curr_hit_point[2]);
               for (; vvit && vfit; ++vvit) {
                  TriMesh::VertexHandle vh1 = *vvit;
                  Eigen::Vector3d proj_end = projectToFacePlane(end_point, (*vfit).idx(), mesh);

                  Eigen::Vector3d q0(mesh.point(vh0)[0], mesh.point(vh0)[1], mesh.point(vh0)[2]);
                  Eigen::Vector3d q1(mesh.point(vh1)[0], mesh.point(vh1)[1], mesh.point(vh1)[2]);

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
            } else {
               TriMesh::HalfedgeHandle crossed_halfedge = mesh.halfedge_handle(mesh.edge_handle(crossed_edge_idx), 0);
               if ((mesh.face_handle(crossed_halfedge).idx()) == std::get<TUPLE_FACE>(prev_point))
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
            Eigen::Vector3d curr_p(curr_hit_point[0], curr_hit_point[1], curr_hit_point[2]);
            p1 = projectToFacePlane(curr_p, next_face_idx, mesh);

            // Place new point inside the triangle
            Eigen::Vector3d point_in_face;
            ACG::Vec3d face_centroid = mesh.calc_face_centroid(mesh.face_handle(next_face_idx));
            Eigen::Vector3d q0(face_centroid[0], face_centroid[1], face_centroid[2]);
            TriMesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(next_face_idx));
            // Intersect the line with segments going from face centroid to face vertices
            for (; fv_it; ++fv_it) {
               ACG::Vec3d edge_point = mesh.point(*fv_it);
               Eigen::Vector3d q1(edge_point[0], edge_point[1], edge_point[2]);

               // As soon as an edge is crossed, stop: we only need one
               if (segmentIntersect(p0, p1, q0, q1, &point_in_face) == INTERSECT_OK &&
                   !(point_in_face - p0).isZero(1e-5) &&
                   !((intersection_point - q0).isZero(1e-5) || (intersection_point - q1).isZero(1e-5))) break;
            }

            // Add new point to recorded path with next face
            recorded_path_.push_front(std::make_tuple(ACG::Vec3d(point_in_face[0], point_in_face[1], point_in_face[2]),
                  next_face_idx, crossed_edge_idx, closestVertexOnFace(crossing,std::get<TUPLE_FACE>(prev_point),mesh)));
         }
      }
      /// Polygonal mesh
      else if (object->dataType(DATA_POLY_MESH)) {
         std::cout << "Warning: no support for drawing curves on polygonal meshes." << std::endl;
         return;
      }
   }
}






