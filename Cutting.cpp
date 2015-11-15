#include "Cutting.hh"


/** \brief Follow path and mark edges for split
 * Edges crossed by the mouse path are stored with the crossing point for later splitting.
 */
void Cutting::markForSplit(BaseObjectData* object) {
   if (recorded_path_.size() < 2 || !object) return;

   while(!recorded_path_.empty()) {
      // Advance until there is a change of face
      std::tuple<ACG::Vec3d,int,int> prev_point = recorded_path_.front();
      recorded_path_.pop_front();
      std::tuple<ACG::Vec3d,int,int> curr_point = recorded_path_.front();
      while (!recorded_path_.empty() && std::get<TUPLE_FACE>(curr_point) == std::get<TUPLE_FACE>(prev_point)) {
         prev_point = curr_point;
         recorded_path_.pop_front();
         curr_point = recorded_path_.front();
      }
      if (recorded_path_.empty()) return;

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
            Eigen::Vector3d intersection_point;
            int crossed_edge_idx = -1;
            TriMesh::HalfedgeHandle crossed_halfedge;
            TriMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(mesh.face_handle(std::get<TUPLE_FACE>(prev_point)));
            for (; fh_it; ++fh_it) {
               TriMesh::Point point_from = mesh.point(mesh.from_vertex_handle(*fh_it));
               Eigen::Vector3d q0(point_from[0], point_from[1], point_from[2]);
               TriMesh::Point point_to = mesh.point(mesh.to_vertex_handle(*fh_it));
               Eigen::Vector3d q1(point_to[0], point_to[1], point_to[2]);

               // Get intersection point
               int intersection_result = segmentIntersect(p0, p1, q0, q1, &intersection_point);
               if (intersection_result == INTERSECT_OK) {
                  crossed_edge_idx = mesh.edge_handle(*fh_it).idx();
                  crossed_halfedge = *fh_it;
                  break;
               } else if (intersection_result == INTERSECT_COLLINEAR) {
                  emit log(LOGWARN, "Lines are collinear: cannot find intersection");
                  /// TODO: What do we do when drawing a cut directly over an edge?
                  break;
               }
            }
            if (crossed_edge_idx == -1) {
               emit log(LOGERR, "Could not find segment intersection");
               continue;
            }

            // Mark edge and crossing point
            ACG::Vec3d crossing(intersection_point[0], intersection_point[1], intersection_point[2]);
            edges_to_split_.push(std::make_pair(crossed_edge_idx, crossing));


            // Project rest of line to next face
            TriMesh::FaceHandle next_face = mesh.face_handle(mesh.opposite_halfedge_handle(crossed_halfedge));
            TriMesh::Normal next_face_normal = mesh.calc_face_normal(next_face);
            Eigen::Vector3d n(next_face_normal[0], next_face_normal[1], next_face_normal[2]);
            p0 = Eigen::Vector3d(intersection_point);
            Eigen::Hyperplane<double,3> plane(n, p0);
            Eigen::Vector3d curr_p(curr_hit_point[0], curr_hit_point[1], curr_hit_point[2]);
            p1 = plane.projection(curr_p);

            // Place new point inside the triangle
            Eigen::Vector3d point_in_face;
            ACG::Vec3d face_centroid = mesh.calc_face_centroid(next_face);
            Eigen::Vector3d q0(face_centroid[0], face_centroid[1], face_centroid[2]);
            TriMesh::FaceVertexIter fv_it = mesh.fv_iter(next_face);
            // Intersect the line with segments going from face centroid to face vertices
            for (; fv_it; ++fv_it) {
               ACG::Vec3d edge_point = mesh.point(*fv_it);
               Eigen::Vector3d q1(edge_point[0], edge_point[1], edge_point[2]);

               // As soon as an edge is crossed, stop: we only need one
               if (segmentIntersect(p0, p1, q0, q1, &point_in_face) == INTERSECT_OK) break;
            }

            // Add new point to recorded path with next face
            recorded_path_.push_front(std::make_tuple(ACG::Vec3d(point_in_face[0], point_in_face[1], point_in_face[2]),
                  next_face.idx(), crossed_edge_idx));
         }
      }
      /// Polygonal mesh
      else if (object->dataType(DATA_POLY_MESH)) {
         emit log(LOGWARN, "No support for polymesh yet");
         return;
      }
   }
}






