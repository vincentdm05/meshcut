#ifndef MESHTOOLS_HH
#define MESHTOOLS_HH

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <math.h>
#include <Eigen/Dense>

class MeshTools
{
private:
   template<typename Vec3Type>
   PolyMesh::Point vecToP(Vec3Type _v);

   template<typename PType>
   Eigen::Vector3d pToVec(PType _p);

   template<typename MeshT>
   Eigen::Vector3d vhToVec(typename MeshT::VertexHandle _h, MeshT* _m);

   Eigen::Vector3d zPlaneRot(double _angle, Eigen::Vector3d _p, Eigen::Vector3d _rot_center);

public:
   MeshTools() {}
   ~MeshTools() {}

   void generateQuadRect(PolyMesh* mesh, size_t _width, size_t _height, bool _tessellate = false);
   void generateTriHex(TriMesh* mesh, size_t _radius, bool _tessellate = false);

   template<typename MeshT>
   void hingedSubdivide(MeshT* mesh);
};

/// TEMPLATED FUNCTIONS DEFINITION

template<typename Vec3Type>
PolyMesh::Point MeshTools::vecToP (Vec3Type _v) { return PolyMesh::Point(_v[0],_v[1],_v[2]); }

template<typename PType>
Eigen::Vector3d MeshTools::pToVec (PType _p) { return Eigen::Vector3d(_p[0],_p[1],_p[2]); }

template<typename MeshT>
Eigen::Vector3d MeshTools::vhToVec(typename MeshT::VertexHandle _h, MeshT* _m)
{ typename MeshT::Point p = _m->point(_h); return pToVec(p); }

template<typename MeshT>
void MeshTools::hingedSubdivide(MeshT* mesh) {
   typename MeshT::FaceIter f_it, f_end(mesh->faces_end());
   for (f_it = mesh->faces_begin(); f_it!=f_end; ++f_it) {
      /// TODO
   }
}





#endif // MESHTOOLS_HH





