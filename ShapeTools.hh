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

   // Handle vertices
   double handleConstraintWeight_;
   std::vector<int> handleIdxs_;
   std::map<int,int> handleConstraintIds_;

   // Edge strain
   double edgeStrainWeight_;
   bool edgeStrainActive_;

   // Triangle strain
   double triangleStrainWeight_;
   bool triangleStrainActive_;

   // Area strain
   double areaStrainWeight_;
   bool areaStrainActive_;

   void setConstraints();
   void moveHandles();

public:
   ShapeTools() : solver_(NULL), update_needed_(true), object_id_(-1), triMesh_(0),
      fixedConstraintWeight_(5.0), fixedVerticesIdx_(),
      handleConstraintWeight_(5.0), handleIdxs_(), handleConstraintIds_(),
      edgeStrainWeight_(1.0), edgeStrainActive_(false),
      triangleStrainWeight_(1.0), triangleStrainActive_(false),
      areaStrainWeight_(1.0), areaStrainActive_(false) {
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

   // Activation of constraints
   enum ConstraintType { EDGE_STRAIN, TRIANGLE_STRAIN, AREA_STRAIN };
   void toggleConstraint(int _constraintType, bool _active) {
      switch (_constraintType) {
      case EDGE_STRAIN:
         edgeStrainActive_ = _active;
         break;
      case TRIANGLE_STRAIN:
         triangleStrainActive_ = _active;
         break;
      case AREA_STRAIN:
         areaStrainActive_ = _active;
         break;
      default:
         break;
      }
   }

   int getObjId() { return object_id_; }

   // Update mesh geometry based on new positions from solving system
   bool solveUpdateMesh();

};

#endif // SHAPETOOLS_HH
