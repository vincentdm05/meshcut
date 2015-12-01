#ifndef MESHGEN_HH
#define MESHGEN_HH

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>

class MeshGen
{
public:
   MeshGen() {}
   ~MeshGen() {}

   void generateQuadRect(PolyMesh* mesh, size_t _width, size_t _height, bool _tesselate = false);
};

#endif // MESHGEN_HH
