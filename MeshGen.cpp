#include "MeshGen.hh"

/**
 * @brief MeshGen::zPlaneRot Rotate a point _p of an angle _angle around _rot_center on the z-plane
 * @param _angle Rotation angle
 * @param _p The point to rotate
 * @param _rot_center The point around which to rotate
 * @return The rotated point
 */
Eigen::Vector3d MeshGen::zPlaneRot(double _angle, Eigen::Vector3d _p, Eigen::Vector3d _rot_center) {
   double a(_angle);
   Eigen::Matrix3d M;
   M << cos(a), -sin(a), 0.0,
         sin(a), cos(a), 0.0,
         0.0, 0.0, 1.0;

   Eigen::Vector3d p = M * (_p-_rot_center) + _rot_center;

   return p;
}

/**
 * @brief MeshGen::generateQuadRect Generate a quad mesh of size _width * _height faces
 * Generate faces from left to right, bottom to top.
 *
 *    .     .
 *    :     :
 *    |     |
 *    x0---x3---...
 *    |     |
 *    x1---x2---...
 *
 * x0 and x3 are shared with the top layer of faces; x2 and x3 are shared with the face
 * on the right.
 *
 * When tessellating, there are two configurations possible for each face. We discriminate
 * even and odd of the sum of indices w and h: (w + h) % 2 is 0 when w and h are both even
 * or odd, and it is 1 when one is even and the other odd.
 *
 * @param mesh The mesh to create a quad to
 * @param _width Number of horizontal faces
 * @param _height Number of vertical faces
 * @param _tessellate Specify whether the mesh should apply hinge tessellation
 */
void MeshGen::generateQuadRect(PolyMesh* mesh, size_t _width, size_t _height, bool _tessellate) {
   double maxwh = _width > _height ? _width : _height;
   double meanw = _width / 2.0;
   double meanh = _height / 2.0;
   bool t(_tessellate);

   const size_t n_faces = _width * _height;
   int w = 0, h = 0;
   std::vector<PolyMesh::VertexHandle> bottomLine;
   PolyMesh::VertexHandle faceSide[2];

   // Build quad mesh incrementally from left to right and bottom to top
   std::vector<PolyMesh::VertexHandle> face;
   for (size_t i=0; i<n_faces; ++i) {
      face.clear();

      // Compute tessellation configuration
      bool even = (w+h)%2 == 0;

      // Build faces in a "U" fashion
      if (w == 0) {
         face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw)/maxwh, (h-meanh+1)/maxwh, 0)));
         if (h == 0) {
            face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw)/maxwh, (h-meanh)/maxwh, 0)));
            face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw+1)/maxwh, (h-meanh)/maxwh, 0)));
         } else {
            if (t) {
               if (even) {
                  face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw)/maxwh, (h-meanh)/maxwh, 0)));
                  face.push_back(bottomLine[0]);
               } else {
                  face.push_back(bottomLine[0]);
                  face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw+1)/maxwh, (h-meanh)/maxwh, 0)));
               }

            } else {
               face.push_back(bottomLine[0]);
               face.push_back(bottomLine[1]);
            }
         }
      } else {
         if (t) {
            if (even) {
               face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw)/maxwh, (h-meanh+1)/maxwh, 0)));
               face.push_back(faceSide[1]);
            } else {
               face.push_back(faceSide[0]);
               if (h == 0) {
                  face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw)/maxwh, (h-meanh)/maxwh, 0)));
               } else {
                  face.push_back(bottomLine[w]);
               }
            }
         } else {
            face.push_back(faceSide[0]);
            face.push_back(faceSide[1]);
         }

         if (h == 0) {
            face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw+1)/maxwh, (h-meanh)/maxwh, 0)));
         } else {
            if (t) {
               if (even) {
                  face.push_back(bottomLine[w]);
               } else {
                  face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw+1)/maxwh, (h-meanh)/maxwh, 0)));
               }
            } else {
               face.push_back(bottomLine[w+1]);
            }
         }
      }
      // Last point doesn't depend on any others
      face.push_back(mesh->add_vertex(PolyMesh::Point((w-meanw+1)/maxwh, (h-meanh+1)/maxwh, 0)));

      if (h == 0) {
         if (t && !even) {
            bottomLine.push_back(face[3]);
         } else {
            bottomLine.push_back(face[0]);
         }
      } else {
         if (t && !even) {
            bottomLine[w] = face[3];
         } else {
            bottomLine[w] = face[0];
         }
      }
      faceSide[0] = face[3];
      faceSide[1] = face[2];

      // Add newly created face to mesh
      mesh->add_face(face);

      ++w;
      // Reset line
      if (w == _width) {
         if (h == 0) {
            bottomLine.push_back(face[3]);
         } else {
            bottomLine[w] = face[3];
         }

         w = 0;
         face.clear();
         ++h;
      }
   }
}

/** \brief Create a hexagon mesh of equilateral triangles with a given radius
 *
 * A radius of 0 yields a single triangle.
 * After that, #t(r) = 6*r^2
 *
 * @param mesh The empty mesh on which to add the triangles
 * @param _radius The radius of the hexagon, i.e. number of edges from the center to the border
 * @param _tessellate Specify whether the mesh should apply hinge tessellation
 */
void MeshGen::generateTriHex(TriMesh *mesh, size_t _radius, bool _tessellate) {
   size_t r(_radius);
   bool t(_tessellate);

   Eigen::Vector3d unitv(1.0, 0.0, 0.0);
   Eigen::Vector3d tri_center(0.0, 0.0, 0.0);

   if (r == 0) {
      std::vector<TriMesh::VertexHandle> triangle;
      triangle.push_back(mesh->add_vertex(vToP(zPlaneRot(-M_PI/6.0, unitv, tri_center))));
      triangle.push_back(mesh->add_vertex(vToP(zPlaneRot(M_PI/2.0, unitv, tri_center))));
      triangle.push_back(mesh->add_vertex(vToP(zPlaneRot(7.0*M_PI/6.0, unitv, tri_center))));

      mesh->add_face(triangle);
   } else {
      size_t level = 0;
      size_t tri_n_at_level = 0;
      size_t n_tris_at_level = 6;

      std::vector<TriMesh::VertexHandle> triangle;
      std::vector<TriMesh::VertexHandle> border;
      TriMesh::VertexHandle face_side[2];

      // Iterate over levels
      for (; level<r; ++level) {
         n_tris_at_level = 6 * (level+1)*(level+1);

         if (level == 0) {
            face_side[0] = mesh->add_vertex(TriMesh::Point(0.0, 0.0, 0.0));
            face_side[1] = mesh->add_vertex(vToP(unitv/r));

            // Iterate over triangles in a level
            for (; tri_n_at_level<n_tris_at_level; ++tri_n_at_level) {
               triangle.clear();

               triangle.push_back(face_side[0]);
               triangle.push_back(face_side[1]);
               if (tri_n_at_level == n_tris_at_level-1) {
                  triangle.push_back(border[0]);
               } else {
                  triangle.push_back(mesh->add_vertex(vToP(zPlaneRot(M_PI/3.0, pToV(mesh->point(triangle.back())), tri_center))));
               }

               border.push_back(triangle[1]);
               face_side[1] = triangle[2];

               mesh->add_face(triangle);
            }
         }
      }
   }
}









