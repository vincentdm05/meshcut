#ifndef CUTTING
#define CUTTING

#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <Eigen/Eigen>
#include <queue>
#include <tuple>

#define TUPLE_POINT 0
#define TUPLE_FACE 1
#define TUPLE_EDGE 2

#define INTERSECT_NO -1
#define INTERSECT_COLLINEAR 0
#define INTERSECT_PARALLEL 1
#define INTERSECT_OK 2

#define PATH_FRONT 0
#define PATH_BACK 1

class Cutting : QObject, LoggingInterface {
   Q_OBJECT
   Q_INTERFACES(LoggingInterface)

signals:
   // LoggingInterface
   void log(Logtype _type, QString _message);
   void log(QString _message);

private:
   typedef std::tuple<ACG::Vec3d,int,int> PathPoint;
   typedef std::list<PathPoint> Path;

   // Determine whether a point lies on a segment
   template<typename VecType>
   bool isOnSegment(VecType p, VecType s0, VecType s1, double prec = 0.05);

   // Get edge between two vertices
   template<typename MeshT>
   int edge_between(typename MeshT::VertexHandle vh_from,
                    typename MeshT::VertexHandle vh_to, MeshT& mesh);

   // Connect adjacent cuts
   template<typename MeshT>
   void connectCuts(typename MeshT::VertexHandle vh, MeshT& mesh);

   // Mouse-recorded path on the mesh: hit point, hit face, closest edge
   Path recorded_path_;

   // Queue of edge handle and crossing point pairs
   std::queue<std::pair<int,ACG::Vec3d> > edges_to_split_;

   // Number of faces over which a cut can be made
   unsigned int face_overs_;

public:
   Cutting() : recorded_path_(0), edges_to_split_(), face_overs_(0) {}
   ~Cutting(){}

   // Draw closest approximation of drawn curve on mesh
   void markForSplit(BaseObjectData* object);

   // Split marked edges and select new applied path
   template<typename MeshT>
   void splitAndSelect(MeshT& mesh);

   // Find if two segments intersect and set intersection_point if appropriate
   template<typename VecType>
   int segmentIntersect(VecType p0, VecType p1,
                         VecType q0, VecType q1, VecType* intersection_point, double prec = 1e-16);

   // Cut along a single edge
   template<typename MeshT>
   bool cutPrimitive(typename MeshT::EdgeHandle edge, MeshT &mesh);

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
 * Thanks to Ronald Goldman, on "Intersection of two lines in three-space" in Graphics Gems 1st ed., p.304
 *
 * Find point of intersection on segment [q0, q1] by half-line [p0, p1).
 * If no intersection is found, false is returned, otherwise intersection_point is set to the point of
 * intersection and true is returned.
 *
 * @param p0 The starting point of the halfline
 * @param p1 The point defining the direction of the halfline
 * @param q0 The start of the segment
 * @param q1 The end of the segment
 * @param intersection_point A pointer to a Eigen::Vector3d object for the intersection point
 * @param prec The precision with which to define collinearity and parallelism
 */
template<typename VecType>
int Cutting::segmentIntersect(VecType p0, VecType p1, VecType q0, VecType q1, VecType *intersection_point, double prec) {

   // Vectors from start to end of each segment
   VecType vp = p1 - p0;
   VecType vq = q1 - q0;

   // Precompute difference and cross product
   VecType diff = q0 - p0;
   VecType cross_prod = vp.cross(vq);

   // Numerators
   double nomT = (diff.cross(vq)).dot(cross_prod);
   double nomS = (diff.cross(vp)).dot(cross_prod);

   // Denominator
   double denom = cross_prod.norm() * cross_prod.norm();

   emit log(LOGOUT, "denom: " + QString::number(denom) + " ;nomS: " + QString::number(nomS) + " ;nomT: " + QString::number(nomT));
   if (denom < prec) {
      if (nomS < prec || nomT < prec) {
         // Lines are collinear
         return INTERSECT_COLLINEAR;
      }
      // Lines are parallel
      return INTERSECT_PARALLEL;
   }

   double t = nomT / denom;
   double s = nomS / denom;

   if (t >= 0 && s >= 0 && s <= 1) {
      *intersection_point = q0 + s * vq;
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
template<typename VecType>
bool Cutting::isOnSegment(VecType p, VecType s0, VecType s1, double prec) {
   // Increase precision by taking furthest point
   VecType segment;
   VecType point_local;
   if ((p-s0).norm() > (p-s1).norm()) {
      segment = s1 - s0;
      point_local = p - s0;
   } else {
      segment = s0 - s1;
      point_local = p - s1;
   }
   VecType cross = point_local.cross(segment);
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
   // Vertices of new edges to select
   std::queue<int> v_edges_to_select;

   // Maps edges to their newly created neighbors after split
   std::map<int,std::set<int> > edges_split;

   // Split
   while (!edges_to_split_.empty()) {
      int edge = edges_to_split_.front().first;
      ACG::Vec3d point = edges_to_split_.front().second;

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
      v_edges_to_select.push(vh.idx());

      // Store index of newly created edge
      /// Assumptions: a new edge has the latest index; TriMesh split performs between 1 and 3
      /// new_edge inserts (depending on boundary conditions), of which our new edge is the first.
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

   // Select
   int prev_vertex_at_split = -1;
   while (!v_edges_to_select.empty()) {
      int curr_vertex_at_split = v_edges_to_select.front();
      if (prev_vertex_at_split != -1) {
         if (mesh.is_trimesh()) {
            //Find connecting edge and select
            int edge_to_select = edge_between(mesh.vertex_handle(prev_vertex_at_split),
                                              mesh.vertex_handle(curr_vertex_at_split), mesh);
            if (edge_to_select != -1)
               mesh.status(mesh.edge_handle(edge_to_select)).set_selected(true);
         } else if (mesh.is_polymesh()) {
            // Create edge
            typename MeshT::HalfedgeHandle new_heh = mesh.new_edge(mesh.vertex_handle(prev_vertex_at_split),
                                                        mesh.vertex_handle(curr_vertex_at_split));
            /// TODO: handle connectivity like in polyconnectivity::split_edge

            // Select new edge
            mesh.status(mesh.edge_handle(new_heh)).set_selected(true);
         }
      }
      prev_vertex_at_split = curr_vertex_at_split;

      v_edges_to_select.pop();
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
bool Cutting::cutPrimitive(typename MeshT::EdgeHandle edge, MeshT& mesh) {
   static_assert((std::is_same<MeshT, TriMesh>::value ||
                 std::is_same<MeshT, PolyMesh>::value),
                 "cutPrimitive: expected either TriMesh or PolyMesh");

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
   connectCuts(vh_up, mesh);
   connectCuts(vh_down, mesh);

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
void Cutting::connectCuts(typename MeshT::VertexHandle vh, MeshT& mesh) {
   static_assert((std::is_same<MeshT, TriMesh>::value ||
                 std::is_same<MeshT, PolyMesh>::value),
                 "connectCuts: expected either TriMesh or PolyMesh");

   /// The new vertex will carry the left side edges
   const size_t _max_cuts = 2;
   // Halfedges at cuts
   typename MeshT::HalfedgeHandle heh_cuts[2*_max_cuts];
   // Halfedges on the left side of the cut
   std::vector<typename MeshT::HalfedgeHandle> left_halfedges;
   bool left_side = true;
   size_t cut = 0;

   // Count cuts and store halfedges according to side
   for (typename MeshT::VertexIHalfedgeIter vih_it = mesh.vih_iter(vh); vih_it.is_valid(); ++vih_it) {
      typename MeshT::HalfedgeHandle heh = *vih_it;
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



#endif // CUTTING

