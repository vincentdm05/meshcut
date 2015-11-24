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
   bool update_needed_;

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
   ShapeTools() : update_needed_(true), object_id_(-1), triMesh_(0),
      fixedConstraintWeight_(10.0), fixedVerticesIdx_(), fixedConstraintIds_(),
      handleConstraintWeight_(10.0), handleIdxs_(), handleConstraintIds_(),
      edgeStrainWeight_(50.0), edgeStrainConstraintIds_() {
      solver_iterations_ = 50;
   }
   ~ShapeTools() { if (solver_ != NULL) delete solver_; }

   void setMesh(TriMesh* _mesh, int _object_id);
   void setMesh(PolyMesh* _mesh, int _object_id) { /** TODO: support for polymesh */ }

   void flagUpdateNeeded() { update_needed_ = true; }
   bool updateNeeded() { return update_needed_; }

   void fixVertices(std::set<int> v_idxs) { fixedVerticesIdx_ = v_idxs; }

   /**
    * @brief setHandles Saves the given handle indices and flags an update if it has changed
    * @param _handleIdxs The handle indices
    */
   void setHandles(std::vector<int> _handleIdxs) {
      if (_handleIdxs != handleIdxs_) {
         handleIdxs_ = _handleIdxs;
         flagUpdateNeeded();
      }
   }

   void setEdgeStrain(double _strainWeight) { edgeStrainWeight_ = _strainWeight; }

   int getObjId() { return object_id_; }

   // Update mesh geometry based on new positions from solving system
   bool solveUpdateMesh();

};

#endif // SHAPETOOLS_HH
