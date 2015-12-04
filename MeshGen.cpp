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

   Eigen::Vector3d p = (M * (_p-_rot_center)) + _rot_center;

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
   double maxwh = (_width > _height ? _width : _height) / 2.0;
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

   mesh->update_normals();
}

/** \brief Create a hexagon mesh of equilateral triangles with a given radius
 *
 * The radiu is the length of the smallest path from the center to the border.
 * A radius of 0 yields a single triangle.
 * After that, #tri(r) = 6*r^2
 *
 *        ...-x9
 *           / \
 *      x3--x2--x8
 *     / \ / \ / \
 *    x4--x0--x1--x7
 *     \ / \ /
 *      x5--x6
 *
 * @param mesh The empty mesh on which to add the triangles
 * @param _radius The radius of the hexagon, i.e. number of edges from the center to the border
 * @param _tessellate Specify whether the mesh should apply hinge tessellation
 */
void MeshGen::generateTriHex(TriMesh *mesh, size_t _radius, bool _tessellate) {
   size_t r(_radius);
   bool t(_tessellate);

   Eigen::Vector3d unitv(1.0, 0.0, 0.0);
   const Eigen::Vector3d tri_origin(0.0, 0.0, 0.0);

   if (r == 0) {
      std::vector<TriMesh::VertexHandle> triangle;
      triangle.push_back(mesh->add_vertex(vecToP(zPlaneRot(-M_PI/6.0, unitv, tri_origin))));
      triangle.push_back(mesh->add_vertex(vecToP(zPlaneRot(M_PI/2.0, unitv, tri_origin))));
      triangle.push_back(mesh->add_vertex(vecToP(zPlaneRot(7.0*M_PI/6.0, unitv, tri_origin))));

      mesh->add_face(triangle);
   } else {
      size_t layer = 0;
      size_t tri_n_on_layer = 0;
      size_t n_tris_on_layer = 6;

      std::vector<TriMesh::VertexHandle> triangle;
      std::vector<TriMesh::VertexHandle> border;
      std::vector<TriMesh::VertexHandle> next_border;
      TriMesh::VertexHandle face_side[2];

      // Iterate over layers
      for (; layer<r; ++layer) {
         // This is the number of total triangles at this layer
         n_tris_on_layer = 6 * (layer+1)*(layer+1);

         // Set border for this iteration
         if (layer > 0) {
            border = next_border;
            next_border.clear();
         }

         // We keep track of the number of even and odd triangles at a layer
         // Note that this count considers parity on a 'side', not on the complete layer
         size_t n_evens_on_side = 0;
         size_t n_odds_on_side = 0;

         if (layer == 0) {
            face_side[0] = mesh->add_vertex(vecToP(tri_origin));
         } else {
            face_side[0] = border[0];
         }
         face_side[1] = mesh->add_vertex(vecToP(vhToVec(face_side[0],mesh) + unitv/r));

         // Keep track of numbers of triangles on a side
         size_t tri_n_on_side = 0;
         size_t n_tris_on_side = 2*(layer+1) - 1;

         bool odd_side = true;
         size_t t_border_access = 1;

         // Iterate over triangles in a layer
         for (; tri_n_on_layer<n_tris_on_layer; ++tri_n_on_layer) {
            triangle.clear();

            // Caution: parity is not based on the index but the vertex number (vertex at idx 0 is number 1, which is odd)
            bool odd_on_side = tri_n_on_side%2 == 0;
            bool odd_on_layer = tri_n_on_layer%2 == 0;

            // First two vertices are connected to the previous triangle, or alternating when tessellation is on
            if (t) {
               if (odd_on_layer) {
                  if (tri_n_on_layer == 0) {
                     triangle.push_back(face_side[0]);
                  } else {
                     if (odd_on_side) {
                        triangle.push_back(mesh->add_vertex(vecToP(vhToVec(face_side[0],mesh))));
                     } else {
                        triangle.push_back(border[t_border_access++]);
                     }
                  }
                  triangle.push_back(face_side[1]);
               } else {
                  triangle.push_back(face_side[0]);
                  triangle.push_back(mesh->add_vertex(vecToP(vhToVec(face_side[1],mesh))));
               }
            } else {
               triangle.push_back(face_side[0]);
               triangle.push_back(face_side[1]);
            }

            // Third vertex is either a new one or connected to the previous layer (except last one)
            if (tri_n_on_layer == n_tris_on_layer-1) {
               // Last triangle on a layer connects to first one
               triangle.push_back(next_border[0]);
            } else {
               if (layer == 0) {
                  triangle.push_back(mesh->add_vertex(vecToP(zPlaneRot(M_PI/3.0, vhToVec(triangle.back(),mesh), tri_origin))));
               } else {
                  if (odd_on_side) {  // Odd triangles on side, starting at 'bend' with triangle #1
                     triangle.push_back(mesh->add_vertex(vecToP(zPlaneRot(M_PI/3.0, vhToVec(triangle[1],mesh), vhToVec(triangle[0],mesh)))));
                     ++n_odds_on_side;
                  } else {    // Even triangles on side
                     ++n_evens_on_side;

                     if (tri_n_on_layer == n_tris_on_layer-2) {   // One before last is connected to border beginning
                        if (t) {
                           triangle.push_back(mesh->add_vertex(vecToP(vhToVec(border[0],mesh))));
                        } else {
                           triangle.push_back(border[0]);
                        }
                     } else {
                        if (t) {
                           if (odd_on_layer) {
                              triangle.push_back(mesh->add_vertex(vecToP(vhToVec(border[t_border_access++],mesh))));
                           } else {
                              triangle.push_back(border[t_border_access++]);
                           }
                        } else {
                           triangle.push_back(border[n_evens_on_side]);
                        }
                     }
                  }
               }
            }

            mesh->add_face(triangle);

            // Prepare for next triangle and next layer
            if (layer == 0) {
               next_border.push_back(triangle[1]);

               if (t) {
                  if (odd_on_layer){
                     next_border.push_back(triangle[2]);
                     face_side[0] = triangle[0];
                  }
               }
               face_side[1] = triangle[2];
            } else {
               // In the even case, no border is recorded
               if (odd_on_side) {
                  next_border.push_back(triangle[1]);
                  // When tessellating, we need to keep track of both top vertices on the border
                  if (t && odd_on_layer) next_border.push_back(triangle[2]);

                  face_side[0] = triangle[0];
                  face_side[1] = triangle[2];
               } else {
                  if (t && odd_on_layer) next_border.push_back(triangle[1]);

                  face_side[0] = triangle[2];
                  face_side[1] = triangle[1];
               }
            }

            // The number of triangles on a side does not exceed twice the number of layers they are at
            tri_n_on_side = (tri_n_on_side + 1) % n_tris_on_side;

            /// TODO: instead of doing that, don't register the underlying border element in the layer before
            if (odd_on_layer && odd_on_side && tri_n_on_side != 0 && tri_n_on_side != 1) ++t_border_access;
            if (tri_n_on_side == 0) odd_side = !odd_side;
         }
      }
   }

   mesh->update_normals();
}









