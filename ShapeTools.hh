#ifndef SHAPETOOLS_HH
#define SHAPETOOLS_HH

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>

#include <Solver.h>
#include <Constraint.h>

#define WEIGHT_MIN 0.0
#define WEIGHT_MAX 10.0
#define WEIGHT_STEP 0.01
#define WEIGHT_DEFAULT 0.1
#define WEIGHT_HANDLE 1.0

#define RANGE_MIN 0.0
#define RANGE_MAX 2.0
#define RANGE_STEP 0.1
#define RANGE_DEFAULT 1.0

#define ANGLE_MIN 0.0
#define ANGLE_MAX M_PI
#define ANGLE_STEP M_PI/180.0
#define ANGLE_DEFAULT M_PI/3.0
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

class ShapeTools {
private:
   ShapeOp::Solver* solver_;
   size_t solver_iterations_;
   bool update_needed_;

   int object_id_;
   TriMesh* triMesh_;
   PolyMesh* polyMesh_;

   // Fixed vertices
   double fixedConstraintWeight_;
   std::set<int> fixedVerticesIdx_;

   // Handle vertices
   double handleConstraintWeight_;
   std::vector<int> handleIdxs_;
   std::map<int,int> handleConstraintIds_;

   // Rigid
   double rigidConstraintWeight_;
   std::vector<int> rigidIdxs_;

   // Edge strain
   double edgeStrainWeight_;
   bool edgeStrainActive_;

   // Triangle strain
   double triangleStrainWeight_;
   bool triangleStrainActive_;

   // Area
   double areaConstraintWeight_;
   double areaMin_;
   double areaMax_;
   bool areaConstraintActive_;

   // Bending
   double bendingConstraintWeight_;
   double bendingMin_;
   double bendingMax_;
   bool bendingConstraintActive_;

   // Rectangle
   double rectConstraintWeight_;
   bool rectConstraintActive_;

   // Angle
   double angleConstraintWeight_;
   double angleMin_;
   double angleMax_;
   bool angleConstraintActive_;

   void setConstraints();
   void moveHandles();

public:
   ShapeTools() : solver_(NULL), update_needed_(true), object_id_(-1), triMesh_(0), polyMesh_(0),
      fixedConstraintWeight_(WEIGHT_MAX), fixedVerticesIdx_(),
      handleConstraintWeight_(WEIGHT_HANDLE), handleIdxs_(), handleConstraintIds_(),
      rigidConstraintWeight_(WEIGHT_MAX), rigidIdxs_(),
      edgeStrainWeight_(WEIGHT_DEFAULT), edgeStrainActive_(false),
      triangleStrainWeight_(WEIGHT_DEFAULT), triangleStrainActive_(false),
      areaConstraintWeight_(WEIGHT_DEFAULT), areaMin_(RANGE_DEFAULT), areaMax_(RANGE_DEFAULT), areaConstraintActive_(false),
      bendingConstraintWeight_(WEIGHT_DEFAULT), bendingMin_(RANGE_DEFAULT), bendingMax_(RANGE_DEFAULT), bendingConstraintActive_(false),
      rectConstraintWeight_(WEIGHT_DEFAULT), rectConstraintActive_(false),
      angleConstraintWeight_(WEIGHT_DEFAULT), angleMin_(ANGLE_MIN), angleMax_(ANGLE_DEFAULT), angleConstraintActive_(false) {
      solver_iterations_ = 50;
   }
   ~ShapeTools() { if (solver_ != NULL) delete solver_; }

   void setMesh(TriMesh* _mesh, int _object_id);
   void setMesh(PolyMesh* _mesh, int _object_id);

   void flagUpdateNeeded() { update_needed_ = true; }
   bool updateNeeded() { return update_needed_; }

   void fixVertices(std::set<int> v_idxs) { fixedVerticesIdx_ = v_idxs; }
   void setRigid(std::vector<int> v_idxs) { rigidIdxs_ = v_idxs; }

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

   void setWeightsAndRanges(double _edgeStrainWeight, double _triangleStrainWeight,
                            double _areaMin, double _areaMax, double _areaWeight,
                            double _bendingMin, double _bendingMax, double _bendingWeight,
                            double _rectWeight,
                            double _angleMin, double _angleMax, double _angleWeight) {
      edgeStrainWeight_ = _edgeStrainWeight;
      triangleStrainWeight_ = _triangleStrainWeight;
      areaMin_ = _areaMin;
      areaMax_ = _areaMax;
      areaConstraintWeight_ = _areaWeight;
      bendingMin_ = _bendingMin;
      bendingMax_ = _bendingMax;
      bendingConstraintWeight_ = _bendingWeight;
      rectConstraintWeight_ = _rectWeight;
      angleMin_ = _angleMin;
      angleMax_ = _angleMax;
      angleConstraintWeight_ = _angleWeight;
   }

   // Activation of constraints
   enum ConstraintType { EDGE_STRAIN, TRIANGLE_STRAIN, AREA, BENDING, RECT, ANGLE };
   void toggleConstraint(int _constraintType, bool _active) {
      switch (_constraintType) {
      case EDGE_STRAIN:
         edgeStrainActive_ = _active;
         break;
      case TRIANGLE_STRAIN:
         triangleStrainActive_ = _active;
         break;
      case AREA:
         areaConstraintActive_ = _active;
         break;
      case BENDING:
         bendingConstraintActive_ = _active;
         break;
      case RECT:
         rectConstraintActive_ = _active;
         break;
      case ANGLE:
         angleConstraintActive_ = _active;
         break;
      default:
         break;
      }
   }

   int getObjId() { return object_id_; }

   // Solve linear system and set the new positions of vertices
   bool solveUpdateMesh();
   bool solveUpdateMesh(size_t _n_iterations);

};

#endif // SHAPETOOLS_HH
