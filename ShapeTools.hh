#ifndef SHAPETOOLS_HH
#define SHAPETOOLS_HH

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>

#include <Solver.h>
#include <Constraint.h>

class ShapeTools {
private:
   ShapeOp::Solver* solver_;

   int object_id_;
   TriMesh* triMesh_;

   // Fixed vertices
   std::set<int> fixedVerticesIdx_;
   std::vector<int> fixedConstraintIds_;

   void updatePositions();

public:
   ShapeTools();
   ~ShapeTools() { delete solver_; }

   void setMesh(TriMesh* _mesh, int _object_id);
   void setMesh(PolyMesh* _mesh, int _object_id) { /** TODO: support for polymesh */ }

   void toggleFixVertices(std::set<int> v_idxs);

   int getObjId() { return object_id_; }

   // Update mesh geometry based on new positions and constraints
   bool updateSolveMesh();

};

#endif // SHAPETOOLS_HH
