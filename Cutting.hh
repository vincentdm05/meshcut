#ifndef CUTTING
#define CUTTING

#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <Eigen/Eigen>
#include <queue>
#include <tuple>

#define TUPLE_POINT 0
#define TUPLE_FACE 1
#define TUPLE_EDGE 2
#define TUPLE_VERTEX 3

#define INTERSECT_NO -1
#define INTERSECT_COLLINEAR 0
#define INTERSECT_PARALLEL 1
#define INTERSECT_OK 2

#define PATH_FRONT 0
#define PATH_BACK 1

class Cutting {

private:
   typedef std::tuple<ACG::Vec3d,int,int,int> PathPoint;
   typedef std::list<PathPoint> Path;

   // Select edges that have been marked
   template<typename MeshT>
   void selectPath(MeshT& mesh);

   // Determine whether a point lies on a segment
   template<typename Vec3Type>
   bool isOnSegment(Vec3Type p, Vec3Type s0, Vec3Type s1, double prec = 0.05);

   // Get edge between two vertices
   template<typename MeshT>
   int edge_between(typename MeshT::VertexHandle vh_from,
                    typename MeshT::VertexHandle vh_to, MeshT& mesh);

   // Get index of vertex connected to a known vertex
   template<typename Vec3Type, typename MeshT>
   int closest_connected_vertex(int vidx_from, Vec3Type p_to, MeshT& mesh);

   // Find closest vertex to hit point on a face
   template<typename Vec3Type, typename MeshT>
   int closestVertexOnFace(Vec3Type _hit_point, int _face_idx, MeshT& mesh);

   // Find point of crossing a face outward
   template<typename Vec3Type, typename MeshT>
   Vec3Type findOutgoingFaceCrossing(const Vec3Type _point_inside, const Vec3Type _point_outside,
                                    int _face_idx, int& crossed_edge_idx, MeshT& mesh, float prec = 1e-10);

   // Given a triangle and a point, project point to plane defined by triangle
   template<typename Vec3Type, typename MeshT>
   Vec3Type projectToFacePlane(Vec3Type _point_to_project, int _face_idx, MeshT mesh);

   // Mouse-recorded path on the mesh: hit point, hit face, closest edge
   Path recorded_path_;

   // Vertices of new edges to select
   std::queue<int> v_path_to_select_;

   // Queue of edge handle and crossing point pairs
   std::queue<std::pair<int,ACG::Vec3d> > edges_to_split_;

   // Number of faces over which a cut can be made
   unsigned int face_overs_;

public:
   Cutting() : recorded_path_(0), v_path_to_select_(), edges_to_split_(), face_overs_(0) {}
   ~Cutting(){}

   // Draw closest approximation of drawn curve on mesh
   void markForSplit(BaseObjectData* object);

   // Split marked edges and select new applied path
   template<typename MeshT>
   void splitAndSelect(MeshT& mesh);

   // Clamp path to closest edges and select resulting path
   template<typename MeshT>
   void clampAndSelect(MeshT& mesh);

   // Find if two segments intersect and set intersection_point if appropriate
   template<typename Vec3Type>
   int segmentIntersect(Vec3Type p0, Vec3Type p1,
                         Vec3Type q0, Vec3Type q1, Vec3Type* intersection_point, double prec = 1e-16);

   // Cut along a single edge
   template<typename MeshT>
   bool cutPrimitive(typename MeshT::EdgeHandle edge, MeshT &mesh, bool joinCuts = true);

   // Connect adjacent cuts
   template<typename MeshT>
   void splitVertex(typename MeshT::VertexHandle vh, MeshT& mesh);

   // Merge a pair of vertices
   template<typename MeshT>
   void mergeVertices(typename MeshT::VertexHandle vh0, typename MeshT::VertexHandle vh1, MeshT& mesh);

   // Merge two meshes
   template<typename MeshT0, typename MeshT1>
   void combineMeshes(MeshT0* mesh0, MeshT1* mesh1);

   /// Setters for mouse-recorded path
   void addPathPoint(PathPoint _point) {
      if (!recorded_path_.empty() && std::get<TUPLE_FACE>(_point) != std::get<TUPLE_FACE>(recorded_path_.back())) {
         ++face_overs_;
      }
      recorded_path_.push_back(_point);
   }

   void clearPath() {
      recorded_path_.clear();
   }

   /// Getters for path
   /**
    * @brief getPathPoint get the point at either side of the path
    * @param side Either PATH_FRONT or PATH_BACK
    * @return A copy of the path point
    */
   PathPoint getPathPoint(int _side = PATH_FRONT) {
      switch(_side) {
      case PATH_BACK:
         return PathPoint(recorded_path_.back());
      default:
         return PathPoint(recorded_path_.front());
      }
   }

   unsigned int pathSize() {
      return recorded_path_.size();
   }

   bool pathEmpty() {
      return recorded_path_.empty();
   }

   // Get number of faces that went under the mouse path
   unsigned int faceOvers() {
      return face_overs_;
   }
};

///////////////////////////////////////////////////////////////////
///////////////// TEMPLATED FUNCTIONS DEFINITION //////////////////
///////////////////////////////////////////////////////////////////


/** \brief Finds intersection point on segment by halfline
 * Concept adapted from "Intersection of two lines in three-space", by Ronald Goldman in Graphics Gems 1st ed., p.304
 *
 * Find point of intersection on segment [q0, q1] by half-line [p0, p1).
 * If an intersection is found, intersection_point is set to the point of intersection.
 * If the lines are collinear, the intersection is set to be the point of the segment that is inside of the
 * half-line.
 *
 * @param p0 The starting point of the halfline
 * @param p1 The point defining the direction of the halfline
 * @param q0 The start of the segment
 * @param q1 The end of the segment
 * @param intersection_point A pointer to a Vec3Type object for the intersection point
 * @param prec The precision with which to define collinearity and parallelism
 * @return INTERSECT_NO | INTERSECT_PARALLEL | INTERSECT_COLLINEAR | INTERSECT_OK
 */
template<typename Vec3Type>
int Cutting::segmentIntersect(Vec3Type _p0, Vec3Type _p1, Vec3Type _q0, Vec3Type _q1, Vec3Type *intersection_point, double prec) {
   Eigen::Vector3d p0(_p0[0], _p0[1], _p0[2]);
   Eigen::Vector3d p1(_p1[0], _p1[1], _p1[2]);
   Eigen::Vector3d q0(_q0[0], _q0[1], _q0[2]);
   Eigen::Vector3d q1(_q1[0], _q1[1], _q1[2]);

   // Vectors from start to end of each segment
   Eigen::Vector3d vp = p1 - p0;
   Eigen::Vector3d vq = q1 - q0;

   // Precompute difference and cross product
   Eigen::Vector3d diff = q0 - p0;
   Eigen::Vector3d cross_prod = vp.cross(vq);

   Eigen::Vector3d diffCrossVq = diff.cross(vq);
   Eigen::Vector3d diffCrossVp = diff.cross(vp);

   // Denominator
   double denom = cross_prod.norm() * cross_prod.norm();

   if (denom < prec) {
      if (diffCrossVp.norm() < prec || diffCrossVq.norm() < prec) {
         // Lines are collinear
         if (isOnSegment(q0, p0, p1)) {
            *intersection_point = Vec3Type(q0[0], q0[1], q0[2]);
         } else {
            *intersection_point = Vec3Type(q1[0], q1[1], q1[2]);
         }
         return INTERSECT_COLLINEAR;
      }
      // Lines are parallel
      return INTERSECT_PARALLEL;
   }

   // Also possible is det(aT; bT; cT) = (a x b) . c
   double t = diffCrossVq.dot(cross_prod) / denom;
   double s = diffCrossVp.dot(cross_prod) / denom;

   if (t >= 0 && s >= 0 && s <= 1) {
      Eigen::Vector3d ip = q0 + s * vq;
      *intersection_point = Vec3Type(ip[0], ip[1], ip[2]);
      return INTERSECT_OK;
   }

   return INTERSECT_NO;
}

/** \brief Determines whether a point is on a segment or not
 *
 * @param p The point to evaluate
 * @param s0 The start of the segment
 * @param s1 The end of the segment
 * @param prec The precision asked of the comparison between cross product and zero
 */
template<typename Vec3Type>
bool Cutting::isOnSegment(Vec3Type _p, Vec3Type _s0, Vec3Type _s1, double prec) {
   Eigen::Vector3d p(_p[0], _p[1], _p[2]);
   Eigen::Vector3d s0(_s0[0], _s0[1], _s0[2]);
   Eigen::Vector3d s1(_s1[0], _s1[1], _s1[2]);

   // Increase precision by taking furthest point
   Eigen::Vector3d segment;
   Eigen::Vector3d point_local;
   if ((p-s0).norm() > (p-s1).norm()) {
      segment = s1 - s0;
      point_local = p - s0;
   } else {
      segment = s0 - s1;
      point_local = p - s1;
   }
   Eigen::Vector3d cross = point_local.cross(segment);
   if (cross.isZero(prec)) {
      double t = point_local.dot(segment) / (segment.norm() * segment.norm());
      // Point is in range
      if (t >= 0 && t <= 1) return true;
   }
   return false;
}

/** \brief Split marked edges and select new applied path
 *
 * Go through all the marked edge-point pair and check whether the edge has already been
 * split before. If it is the case, find the correct edge index and split.
 * For all the new vertices created from split, connect (for PolyMesh) them and select the
 * new edge.
 *
 * @param mesh The mesh
 */
template<typename MeshT>
void Cutting::splitAndSelect(MeshT &mesh) {

   // Maps edges to their newly created neighbors after split
   std::map<int,std::set<int> > edges_split;

   // Split
   while (!edges_to_split_.empty()) {
      int edge = edges_to_split_.front().first;
      ACG::Vec3d point = edges_to_split_.front().second;

      // Manage case where no split is needed
      if (edge == -1) {
         /// TODO: segfault here
         int v_idx = closest_connected_vertex(v_path_to_select_.back(), point, mesh);
         v_path_to_select_.push(v_idx);
         edges_to_split_.pop();
         continue;
      }

      // Create vector for edge map
      if (edges_split.find(edge) == edges_split.end()) {
         std::set<int> new_edge;
         new_edge.insert(edge);
         edges_split[edge] = new_edge;
      }

      // Query the edge map for the correct edge
      int edge_to_split = edge;
      std::set<int> edges = edges_split[edge];
      Eigen::Vector3d p(point[0], point[1], point[2]);
      for (std::set<int>::iterator it(edges.begin()); it!=edges.end(); ++it) {
         // Find out if closest edge is still the one to split
         typename MeshT::HalfedgeHandle heh = mesh.halfedge_handle(mesh.edge_handle(*it), 0);
         ACG::Vec3d segment_start = mesh.point(mesh.from_vertex_handle(heh));
         Eigen::Vector3d s0(segment_start[0], segment_start[1], segment_start[2]);
         ACG::Vec3d segment_end = mesh.point(mesh.to_vertex_handle(heh));
         Eigen::Vector3d s1(segment_end[0], segment_end[1], segment_end[2]);
         if (isOnSegment(p, s0, s1, 0.05)) {
            edge_to_split = *it;
            break;
         }
      }

      // Split edge
      typename MeshT::VertexHandle vh = mesh.add_vertex(point);
      mesh.split(mesh.edge_handle(edge_to_split), vh);
      --face_overs_;
      v_path_to_select_.push(vh.idx());

      // Store index of newly created edge
      /// Assumptions: a new edge has the latest index; TriMesh split performs between 1 and 3
      /// new_edge inserts (depending on boundary conditions), of which our new edge is the first.
      /// PolyMesh doesn't insert new edges on split, other than the one on the edge split.
      int new_edge_idx = mesh.n_edges() - 1;
      if (mesh.is_trimesh()) {
         typename MeshT::HalfedgeHandle heh0 = mesh.halfedge_handle(mesh.edge_handle(edge), 0);
         if (!mesh.is_boundary(heh0)) --new_edge_idx;
         typename MeshT::HalfedgeHandle heh1 = mesh.halfedge_handle(mesh.edge_handle(edge), 1);
         if (!mesh.is_boundary(heh1)) --new_edge_idx;
      }
      edges_split[edge].insert(new_edge_idx);

      edges_to_split_.pop();
   }
   edges_split.clear();

   mesh.update_normals();

   // Select
   selectPath(mesh);
}

/** \brief Clamp curve to closest edges and select resulting path
 *
 */
template<typename MeshT>
void Cutting::clampAndSelect(MeshT& mesh) {
   if (recorded_path_.size() < 2) {
      recorded_path_.clear();
      return;
   }

   // This is an important safeguard to prevent a path from oscillating indefinitely when reconstructing from a jump
   std::set<int> jump_visited;

   // Get first point of curve
   v_path_to_select_.push(std::get<TUPLE_VERTEX>(recorded_path_.front()));

   while (recorded_path_.size() > 1) {
      // Advance until there is a change of vertex
      PathPoint curr_point = recorded_path_.front();
      recorded_path_.pop_front();
      PathPoint next_point = recorded_path_.front();
      while (recorded_path_.size() > 1 && std::get<TUPLE_VERTEX>(curr_point) == std::get<TUPLE_VERTEX>(next_point)) {
         curr_point = next_point;
         recorded_path_.pop_front();
         next_point = recorded_path_.front();
      }
      if (recorded_path_.size() < 2 && std::get<TUPLE_VERTEX>(curr_point) == std::get<TUPLE_VERTEX>(next_point)) break;

      // Get the two points and their closest edge
      int curr_edge = std::get<TUPLE_EDGE>(curr_point);
      int next_edge = std::get<TUPLE_EDGE>(next_point);
      int next_path_vertex = std::get<TUPLE_VERTEX>(next_point);

      // Reset jump memory
      if (curr_edge != -1) jump_visited.clear();

      /// Check if next edge is connected to next vertex, if not skip.
      /// This is due to obtuse or right triangles, where the path gets closer to the opposite vertex when following an edge.
      bool isConnected = false;
      typename MeshT::VertexEdgeIter ve_it = mesh.ve_iter(mesh.vertex_handle(next_path_vertex));
      for (; ve_it.is_valid(); ++ve_it) {
         if (ve_it->idx() == next_edge) {
            isConnected = true;
         }
      }
      if (!isConnected) {
         continue;
      }

      // Vertex path registration
      if (curr_edge == next_edge) {
         v_path_to_select_.push(next_path_vertex);
      }
      // Jump
      else {
         typename MeshT::VertexHandle current_vertex = mesh.vertex_handle(std::get<TUPLE_VERTEX>(curr_point));
         jump_visited.insert(current_vertex.idx());

         // Find closest current vertex 1-ring neighbour to next vertex
         ACG::Vec3d next_v_point = mesh.point(mesh.vertex_handle(next_path_vertex));
         typename MeshT::VertexVertexIter vv_it = mesh.vv_iter(current_vertex);
         double min_dist = std::numeric_limits<double>::max();
         int candidate_next_vertex_idx = -1;
         for (; vv_it.is_valid(); ++vv_it) {
            // Avoid falling twice on the same vertex during the same jump
            if (jump_visited.find(vv_it->idx()) == jump_visited.end()) {
               double dist = (next_v_point - mesh.point(*vv_it)).norm();
               if (dist < min_dist) {
                  min_dist = dist;
                  candidate_next_vertex_idx = (*vv_it).idx();
               }
            }
         }
         // If no vertex was found, move to the next vertex
         if (min_dist == std::numeric_limits<double>::max()) continue;

         // Record vertex
         v_path_to_select_.push(candidate_next_vertex_idx);
         // Next iteration's current vertex is this iteration's current candidate
         recorded_path_.push_front(std::make_tuple(ACG::Vec3d(), -1, -1, candidate_next_vertex_idx));
      }
   }

   recorded_path_.clear();

   // Select
   selectPath(mesh);
}

/** \brief Select edges on the path that has been marked
 * Go through the list v_path_to_select_, connect and select found edges.
 *
 * @param mesh The mesh
 */
template<typename MeshT>
void Cutting::selectPath(MeshT& mesh) {
   if (v_path_to_select_.size()<2) return;

   int curr_vertex = -1;
   while (!v_path_to_select_.empty()) {
      int next_vertex = v_path_to_select_.front();
      if (curr_vertex != -1) {
         //Find connecting edge and select
         int edge_to_select = edge_between(mesh.vertex_handle(curr_vertex),
                                           mesh.vertex_handle(next_vertex), mesh);
         if (edge_to_select != -1) {
            mesh.status(mesh.edge_handle(edge_to_select)).set_selected(true);
         } else {
            std::set<int> v_visited;
            while (curr_vertex != next_vertex) {
               v_visited.insert(curr_vertex);

               // Find closest current vertex 1-ring neighbour to next vertex
               ACG::Vec3d next_v_point = mesh.point(mesh.vertex_handle(next_vertex));
               typename MeshT::VertexVertexIter vv_it = mesh.vv_iter(mesh.vertex_handle(curr_vertex));
               double min_dist = std::numeric_limits<double>::max();
               int candidate_next_vertex_idx = -1;
               for (; vv_it.is_valid(); ++vv_it) {
                  // Avoid falling twice on the same vertex during the same jump
                  if (v_visited.find(vv_it->idx()) == v_visited.end()) {
                     double dist = (next_v_point - mesh.point(*vv_it)).norm();
                     if (dist < min_dist) {
                        min_dist = dist;
                        candidate_next_vertex_idx = (*vv_it).idx();
                     }
                  }
               }

               // Connect
               if (candidate_next_vertex_idx != -1) {
                  edge_to_select = edge_between(mesh.vertex_handle(curr_vertex),
                                                mesh.vertex_handle(candidate_next_vertex_idx), mesh);
                  mesh.status(mesh.edge_handle(edge_to_select)).set_selected(true);
                  curr_vertex = candidate_next_vertex_idx;
               } else {
                  break;
               }
            }
         }
      }
      curr_vertex = next_vertex;

      v_path_to_select_.pop();
   }
}

/** \brief Cut along edge
 *
 * Look at the surrounding topology of the edge and cut along it.
 * The following schematic shows how the middle edge relates to its
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
 * @return success state
 */
template<typename MeshT>
bool Cutting::cutPrimitive(typename MeshT::EdgeHandle edge, MeshT& mesh, bool joinCuts) {
   // Already a hole there
   if (mesh.is_boundary(edge)) {
      return false;
   }

   /// Get relevant neighbouring components
   // Original edge's halfedges
   typename MeshT::HalfedgeHandle heh_in = mesh.halfedge_handle(edge, 0);
   typename MeshT::HalfedgeHandle heh_out = mesh.halfedge_handle(edge, 1);

   // Left face halfedges
   typename MeshT::HalfedgeHandle left_up_heh_to_new = mesh.next_halfedge_handle(heh_out);
   typename MeshT::HalfedgeHandle left_down_heh_to_new = mesh.prev_halfedge_handle(heh_out);

   // Face adjacent to new edge
   typename MeshT::FaceHandle fh_left = mesh.face_handle(heh_out);

   // Edge endpoints
   typename MeshT::VertexHandle vh_up = mesh.from_vertex_handle(heh_in);
   typename MeshT::VertexHandle vh_down = mesh.to_vertex_handle(heh_in);


   /// Create new edge, copy all properties and adjust pointers
   // Outward new halfedge
   typename MeshT::HalfedgeHandle new_heh_out = mesh.new_edge(vh_down, vh_up);
   mesh.copy_all_properties(heh_in, new_heh_out, true);
   mesh.set_vertex_handle(new_heh_out, vh_down);
   mesh.set_next_halfedge_handle(new_heh_out, heh_out);
   mesh.set_next_halfedge_handle(heh_out, new_heh_out);
   // no need to set_prev_halfedge_handle as it is done in set_next_halfedge_handle
   mesh.set_halfedge_handle(vh_up, new_heh_out);

   // Inward new halfedge
   typename MeshT::HalfedgeHandle new_heh_in = mesh.opposite_halfedge_handle(new_heh_out);
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
   if (joinCuts) {
      splitVertex(vh_up, mesh);
      splitVertex(vh_down, mesh);
   }

   mesh.update_normals();

   return true;
}

/** \brief Get the index of the edge between two given vertices
 *
 * @param vh_from the first vertex
 * @param vh_to the second vertex
 * @param mesh the mesh on which reside the vertices and edge
 * @return the index of the edge, or -1 if not found
 */
template<typename MeshT>
int Cutting::edge_between(typename MeshT::VertexHandle vh_from,
                          typename MeshT::VertexHandle vh_to, MeshT& mesh) {
   typename MeshT::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh_from);
   for (; voh_it.is_valid(); ++voh_it) {
      if (mesh.to_vertex_handle(*voh_it) == vh_to) {
         return mesh.edge_handle(*voh_it).idx();
      }
   }
   return -1;
}

/** \brief Get index of vertex connected to a known vertex
 *
 * @param vh_from A handle to a known vertex
 * @param p_to The point for which to find the closest vertex index
 * @param mesh The mesh
 * @return The index of the closest vertex around the known vertex to the given point
 */
template<typename Vec3Type, typename MeshT>
int Cutting::closest_connected_vertex(int vidx_from, Vec3Type p_to, MeshT& mesh) {
   int v_idx = vidx_from;
   typename MeshT::Point point_to(p_to[0], p_to[1], p_to[2]);
   float min_dist = (mesh.point(mesh.vertex_handle(vidx_from)) - point_to).norm();

   typename MeshT::VertexOHalfedgeIter voh_it = mesh.voh_iter(mesh.vertex_handle(vidx_from));
   for (; voh_it.is_valid(); ++voh_it) {
      typename MeshT::Point v_point = mesh.point(mesh.to_vertex_handle(*voh_it));
      float dist = (v_point - point_to).norm();
      if (dist < min_dist) {
         min_dist = dist;
         v_idx = (mesh.to_vertex_handle(*voh_it)).idx();
      }
   }

   return v_idx;
}

/** \brief Joins adjacent cuts
 *
 * If two adjacent cuts are detected, the edge will be split and the properties and pointers
 * of connected components will be updated.
 *
 *  Before:
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
 *  After:
 *                x             x
 *                |           /
 *      left      |>  cut  </     right
 *      side      |       /      side
 *                |     /
 *                |   /
 *     x----------x  x vh
 *         new_vh |   \
 *                |     \
 *                |       \
 *                |         \
 *                |           \
 *                x             x
 *
 * @param vh The connecting vertex
 * @param mesh The mesh
 */
template<typename MeshT>
void Cutting::splitVertex(typename MeshT::VertexHandle vh, MeshT& mesh) {
   /// The new vertex will carry the left side edges
   const size_t max_cuts = 2;
   // Halfedges at cuts
   typename MeshT::HalfedgeHandle heh_cuts[2*max_cuts];
   // Halfedges on the left side of the cut
   std::vector<typename MeshT::HalfedgeHandle> left_halfedges;
   bool left_side = true;
   size_t cut = 0;

   // Count cuts and store halfedges according to side
   for (typename MeshT::VertexIHalfedgeIter vih_it = mesh.vih_iter(vh); vih_it.is_valid(); ++vih_it) {
      typename MeshT::HalfedgeHandle heh = *vih_it;
      if (mesh.is_boundary(heh) && cut < max_cuts) {
         heh_cuts[2*cut] = heh;
         heh_cuts[2*cut+1] = mesh.next_halfedge_handle(heh);
         ++cut;
         left_side = !left_side;
      } else if (left_side) {
         left_halfedges.push_back(heh);
      }
   }

   // Number of cuts must not exceed _max_cuts
   if (cut > max_cuts) {
      std::cerr << "Vertex is connected to more than " << max_cuts << " cuts." << std::endl;
   } else if (cut == max_cuts) {
      // Split vertex
      typename MeshT::VertexHandle new_vh = mesh.add_vertex(mesh.point(vh));
      mesh.copy_all_properties(vh, new_vh, true);

      /// Update pointers
      // Left vertex boundary halfedges
      mesh.set_next_halfedge_handle(heh_cuts[0], heh_cuts[3]);
      mesh.set_halfedge_handle(new_vh, heh_cuts[3]);
      mesh.set_vertex_handle(heh_cuts[0], new_vh);
      // Other left halfedges
      if (!left_halfedges.empty()) {
         typename std::vector<typename MeshT::HalfedgeHandle>::iterator heh_it = left_halfedges.begin();
         for (; heh_it != left_halfedges.end(); ++heh_it) {
            mesh.set_vertex_handle(*heh_it, new_vh);
         }
      }

      // Right vertex
      mesh.set_next_halfedge_handle(heh_cuts[2], heh_cuts[1]);
      mesh.set_halfedge_handle(vh, heh_cuts[1]);
      mesh.set_vertex_handle(heh_cuts[2], vh);
      // No need to rewire right side halfedges
   }
}

/** \brief Merge two vertices into one
 *
 * Assumption: both vertices lie on a boundary
 *
 *  Before:
 *                x             x   Up
 *                |           /
 *      left      |>  cut  </     right
 *      side      |       /      side
 *                |     /
 *                |   /
 *     x----------x  x vh1
 *            vh0 |   \
 *                |     \
 *                |       \
 *                |         \
 *                |           \
 *                x             x   Down
 *
 *  After:
 *                x          x   Up
 *                |> cut  </
 *      left      |      /     right
 *      side      |    /      side
 *                |  /
 *                |/
 *     x----------x vh0
 *                |\
 *                |  \
 *                |    \
 *                |      \
 *                |> cut  <\
 *                x          x   Down
 *
 * @param vh0 The first vertex handle
 * @param vh1 The second vertex handle
 * @param mesh The mesh
 */
template<typename MeshT>
void Cutting::mergeVertices(typename MeshT::VertexHandle vh0, typename MeshT::VertexHandle vh1, MeshT& mesh) {
   if (mesh.status(vh0).deleted() || mesh.status(vh1).deleted()) return;
   if (!mesh.is_boundary(vh0) || !mesh.is_boundary(vh1)) return;

   // Record boundary halfedges
   bool isOnBorderLeft, isOnBorderRight;
   isOnBorderLeft = isOnBorderRight = false;
   typename MeshT::HalfedgeHandle heh_border_left_up, heh_border_right_down;
   typename MeshT::VertexIHalfedgeIter vih_it = mesh.vih_iter(vh0);
   for (; vih_it.is_valid(); ++vih_it) {
      if (mesh.is_boundary(*vih_it)) {
         heh_border_left_up = *vih_it;
         isOnBorderLeft = true;
      }
   }
   vih_it = mesh.vih_iter(vh1);
   for (; vih_it.is_valid(); ++vih_it) {
      if (mesh.is_boundary(*vih_it)) {
         heh_border_right_down = *vih_it;
         isOnBorderRight = true;
      }
   }

   if (isOnBorderLeft && isOnBorderRight) {
      typename MeshT::HalfedgeHandle heh_border_left_down = mesh.next_halfedge_handle(heh_border_left_up);
      typename MeshT::HalfedgeHandle heh_border_right_up = mesh.next_halfedge_handle(heh_border_right_down);

      // Rewire right side edges to left side vertex
      typename MeshT::VertexIHalfedgeIter vih_it_right = mesh.vih_iter(vh1);
      for (; vih_it_right.is_valid(); ++vih_it_right) {
         mesh.set_vertex_handle(*vih_it_right, vh0);
      }

      // Connect borders
      mesh.set_next_halfedge_handle(heh_border_left_up, heh_border_right_up);
      mesh.set_next_halfedge_handle(heh_border_right_down, heh_border_left_down);

      // Move merged vertex to center of gravity
      typename MeshT::Point p0, p1, p;
      p0 = mesh.point(vh0);
      p1 = mesh.point(vh1);
      p = typename MeshT::Point((p0[0]+p1[0])/2.0, (p0[1]+p1[1])/2.0, (p0[2]+p1[2])/2.0);
      mesh.set_point(vh0, p);

      // Finally, remove right side vertex
      mesh.set_halfedge_handle(vh1, typename MeshT::HalfedgeHandle());
      mesh.delete_vertex(vh1);
   }
}

/** \brief Find closest vertex to point on a face
 *
 * @param _point The point on the face
 * @param _face_idx The index of the face
 * @param mesh The mesh
 * @return The index of the closest vertex
 */
template<typename Vec3Type, typename MeshT>
int Cutting::closestVertexOnFace(Vec3Type _point, int _face_idx, MeshT& mesh) {
   typename MeshT::Point point(_point[0], _point[1], _point[2]);
   typename MeshT::FaceHandle faceh = mesh.face_handle(_face_idx);

   // Get all vertices
   typename std::vector<typename MeshT::VertexHandle> vertexHandles;
   typename MeshT::FaceVertexIter fv_it(mesh, faceh);
   for (; fv_it.is_valid(); ++fv_it) {
      vertexHandles.push_back(*fv_it);
   }

   // Find closest Vertex
   typename std::vector<typename MeshT::VertexHandle>::iterator v_it(vertexHandles.begin());
   typename MeshT::VertexHandle vertex(*v_it++);
   double min_v_dist = (mesh.point(vertex) - point).norm();
   for (; v_it != vertexHandles.end(); ++v_it) {
      double v_dist = (mesh.point(*v_it) - point).norm();
      if (v_dist < min_v_dist) {
         min_v_dist = v_dist;
         vertex = *v_it;
      }
   }

   return vertex.idx();
}

/** \brief Find point of crossing a face outward
 *
 * Given a face, we iterate over its edges and compute the intersection with the half-line
 * defined by [_point_inside, _point_outside). The return value is set to be the crossing
 * of this half-line with one of the edges and crossed_edge_idx is set to the index of the
 * crossed edge.
 * Note that is does not matter if _point_inside lies on top of an edge as this intersection
 * will simply be ignored.
 * If the half-line is outside of the face and doesn't cross it at any edge, the return
 * value as well as crossed_edges_idx will not be set.
 *
 * @param _point_inside The point inside the face
 * @param _point_outside The point outside the face
 * @param _face_idx The index of the face
 * @param crossed_edge_idx [out] The index of the crossed edge if an intersection
 *    was found successfully, or -1 if not
 * @param mesh The mesh
 * @return The intersection point
 */
template<typename Vec3Type, typename MeshT>
Vec3Type Cutting::findOutgoingFaceCrossing(const Vec3Type _point_inside, const Vec3Type _point_outside,
                                          int _face_idx, int& crossed_edge_idx, MeshT& mesh, float prec) {
   Eigen::Vector3d point_inside(_point_inside[0], _point_inside[1], _point_inside[2]);
   Eigen::Vector3d point_outside(_point_outside[0], _point_outside[1], _point_outside[2]);

   // Find crossed edge and point
   Eigen::Vector3d intersection_point(0.0);
   typename MeshT::FaceHalfedgeIter fh_it = mesh.fh_iter(mesh.face_handle(_face_idx));
   for (; fh_it; ++fh_it) {
      typename MeshT::Point point_from = mesh.point(mesh.from_vertex_handle(*fh_it));
      Eigen::Vector3d q0(point_from[0], point_from[1], point_from[2]);
      typename MeshT::Point point_to = mesh.point(mesh.to_vertex_handle(*fh_it));
      Eigen::Vector3d q1(point_to[0], point_to[1], point_to[2]);

      // Get intersection point
      /// TODO: analyse needed precisions in segmentIntersect to be able to avoid same point accurately afterwards
      int intersection_result = segmentIntersect(point_inside, point_outside, q0, q1, &intersection_point);

      // Avoid same point if entry point was on an edge
      if ((point_inside - intersection_point).isZero(prec)) continue;

      if (intersection_result == INTERSECT_OK) {
         crossed_edge_idx = mesh.edge_handle(*fh_it).idx();
         break;
      } else if (intersection_result == INTERSECT_COLLINEAR) {
         crossed_edge_idx = -1;
         break;
      }
   }

   return Vec3Type(intersection_point[0], intersection_point[1], intersection_point[2]);
}

/** \brief Given a triangle and a point, project point to plane defined by triangle
 *
 * @param _point_to_project The point to project on the plane
 * @param _face_idx The index of the face that defines the plane
 * @param _mesh The mesh
 * @return The projected point
 */
template<typename Vec3Type, typename MeshT>
Vec3Type Cutting::projectToFacePlane(Vec3Type _point_to_project, int _face_idx, MeshT _mesh) {
   typename MeshT::FaceHandle fh = _mesh.face_handle(_face_idx);
   typename MeshT::Normal face_normal = _mesh.calc_face_normal(fh);
   Eigen::Vector3d n(face_normal[0], face_normal[1], face_normal[2]);

   Eigen::Vector3d p(_point_to_project[0], _point_to_project[1], _point_to_project[2]);
   typename MeshT::Point face_centroid = _mesh.calc_face_centroid(fh);
   Eigen::Vector3d face_p(face_centroid[0], face_centroid[1], face_centroid[2]);

   Eigen::Hyperplane<double,3> plane(n, face_p);
   Eigen::Vector3d projection = plane.projection(p);

   return Vec3Type(projection[0], projection[1], projection[2]);
}

/** \brief Merge two given meshes into one
 * If the second mesh is a polymesh and the first a triangle mesh, it is the second that will carry the combined mesh.
 *
 * @param mesh0 The first mesh
 * @param mesh1 The second mesh
 */
template<typename MeshT0, typename MeshT1>
void Cutting::combineMeshes(MeshT0* mesh0, MeshT1* mesh1) {
   if (mesh0 == NULL || mesh1 == NULL) return;
   if (mesh0->n_vertices() == 0 || mesh1->n_vertices() == 0) return;

   if (mesh0->is_trimesh() && mesh1->is_polymesh()) {   // Merged mesh is in the second original mesh
      combineMeshes(mesh1, mesh0);

   } else { // Merged mesh is in the first original mesh
      std::map<typename MeshT0::VertexHandle, typename MeshT0::VertexHandle> vmap;

      // Add new vertices
      typename MeshT1::VertexIter v_it, v_end(mesh1->vertices_end());
      for (v_it = mesh1->vertices_begin(); v_it!=v_end; ++v_it) {
         typename MeshT0::VertexHandle vh = mesh0->add_vertex(typename MeshT1::Point(mesh1->point(*v_it)));
         vmap[*v_it] = vh;
      }

      // Add faces
      std::vector<typename MeshT0::VertexHandle> face;
      typename MeshT1::FaceIter f_it, f_end(mesh1->faces_end());
      for (f_it = mesh1->faces_begin(); f_it!=f_end; ++f_it) {
         typename MeshT1::FaceVertexIter fv_it = mesh1->fv_iter(*f_it);
         for (; fv_it; ++fv_it) {
            face.push_back(vmap[*fv_it]);
         }
         mesh0->add_face(face);

         face.clear();
      }

      // Update first mesh
      mesh0->update_normals();
   }
}





#endif // CUTTING
