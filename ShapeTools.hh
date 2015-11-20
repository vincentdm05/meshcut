#ifndef SHAPETOOLS_HH
#define SHAPETOOLS_HH

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>

#include <Solver.h>
#include <Constraint.h>

class ShapeTools {
private:
   ShapeOp::Solver* solver_;
   size_t solver_iterations_;
   bool is_running_;

   int object_id_;
   TriMesh* triMesh_;

   // Fixed vertices
   double fixedConstraintWeight_;
   std::set<int> fixedVerticesIdx_;
   std::vector<int> fixedConstraintIds_;

   // Handle vertex
   double handleConstraintWeight_;
   std::vector<int> handleIdxs_;
   std::map<int,int> handleConstraintIds_;

   // Edge strain
   double edgeStrainWeight_;
   std::vector<int> edgeStrainConstraintIds_;

   void setConstraints();
   void moveHandles();

public:
   ShapeTools();
   ~ShapeTools() { delete solver_; }

   void setMesh(TriMesh* _mesh, int _object_id);
   void setMesh(PolyMesh* _mesh, int _object_id) { /** TODO: support for polymesh */ }

   void playPause() { is_running_ = !is_running_; if (is_running_) updateMesh(); }
   bool isRunning() { return is_running_; }

   void updateMesh() { setMesh(triMesh_, object_id_); }

   void toggleFixVertices(std::set<int> v_idxs);
   void setHandles(std::vector<int> _handleIdxs) { handleIdxs_ = _handleIdxs; }

   void setEdgeStrain(int _strainWeight) { edgeStrainWeight_ = _strainWeight; }

   int getObjId() { return object_id_; }

   // Update mesh geometry based on new positions from solving system
   void solveUpdateMesh();

};

#endif // SHAPETOOLS_HH
