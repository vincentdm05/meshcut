#include "MeshGen.hh"

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
 * When tesselating, there are two configurations possible for each face. We discriminate
 * even and odd of the sum of indices w and h: (w + h) % 2 is 0 when w and h are both even
 * or odd, and it is 1 when one is even and the other odd.
 *
 * @param mesh The mesh to create a quad to
 * @param _width Number of horizontal faces
 * @param _height Number of vertical faces
 */
void MeshGen::generateQuadRect(PolyMesh* mesh, size_t _width, size_t _height, bool _tesselate) {
   double maxwh = _width > _height ? _width : _height;
   double meanw = _width / 2.0;
   double meanh = _height / 2.0;
   bool t(_tesselate);

   const size_t n_faces = _width * _height;
   int w = 0, h = 0;
   std::vector<PolyMesh::VertexHandle> bottomLine;
   PolyMesh::VertexHandle faceSide[2];

   // Build quad mesh incrementally from left to right and bottom to top
   std::vector<PolyMesh::VertexHandle> face;
   for (size_t i=0; i<n_faces; ++i) {
      face.clear();

      // Compute tesselation configuration
      bool even = (w+h)%2 == 1;

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
