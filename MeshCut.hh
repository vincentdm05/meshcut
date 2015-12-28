#ifndef MESHCUT_HH
#define MESHCUT_HH

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/MouseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolbarInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/PickingInterface.hh>
#include <OpenFlipper/BasePlugin/BackupInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ACG/Scenegraph/LineNode.hh>
#include <eigen3/Eigen/Geometry>

#include <queue>
#include <tuple>

#include "Cutting.hh"
#include "ShapeTools.hh"
#include "MeshTools.hh"

class MeshCut : public QObject, BaseInterface, MouseInterface, ToolbarInterface, ToolboxInterface, LoggingInterface, PickingInterface, BackupInterface, LoadSaveInterface {
   Q_OBJECT
   Q_INTERFACES(BaseInterface)
   Q_INTERFACES(MouseInterface)
   Q_INTERFACES(ToolbarInterface)
   Q_INTERFACES(ToolboxInterface)
   Q_INTERFACES(LoggingInterface)
   Q_INTERFACES(PickingInterface)
   Q_INTERFACES(BackupInterface)
   Q_INTERFACES(LoadSaveInterface)

#if QT_VERSION >= 0x050000
  Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-MeshCut")
#endif

   signals:
      // BaseInterface
      void updateView();
      void updatedObject(int _id, const UpdateType& _type);
      // ToolbarInterface
      void addToolbar(QToolBar* _toolbar);
      // ToolboxInterface
      void addToolbox( QString _name  , QWidget* _widget, QIcon* _icon );
      // LoggingInterface
      void log(Logtype _type, QString _message);
      void log(QString _message);
      // PickingInterface
      void addHiddenPickMode( const std::string& _mode );
      // BackupInterface
      void createBackup( int _objectid, QString _name, UpdateType _type = UPDATE_ALL );
      // LoadSaveInterface
      void addEmptyObject( DataType _type, int& _id );
      void deleteObject( int _id );

   private slots:
      // BaseInterface
      void initializePlugin();
      void pluginsInitialized();
      // PickingInterface
      void slotPickModeChanged( const std::string& _mode);
      // MouseInterface
      void slotMouseEvent(QMouseEvent* _event);
      // Called when another plugin performs a change on an object
      void slotObjectUpdated(int _id, const UpdateType& _type);
      // Backup has been restored
      void slotRestored(int _objectid);

      // Called when toolbar is clicked
      void toolBarTriggered(QAction* _action);

      /// Cutting tools slots
      // Cut mode selection
      void slotSelectEdgesButtonClicked();
      void slotDrawButtonClicked();
      void slotCutSelectedEdges();
      void slotSelectVerticesButtonClicked();
      void slotMergeSelected();
      void slotSplitVertex();
      void slotCombineMeshes();

      /// Shape tools slots
      void slotUseShapeToolsCheckBoxToggled();
      void slotFixSelectedVertices();
      void slotRigidify();
      void slotFlagUpdate();
      void slotConstraintCheckBoxToggled();
      void slotUpdateMesh();
      void slotNSolverIterationsChanged();

      /// Mesh generation slots
      void slotQuadGen();
      void slotTriGen();

   public slots:
      QString version() { return QString("1.0"); }

   private:
      // Sets active mesh, edge and vertex
      BaseObjectData *setActiveElements(QPoint _pos);
      // Find selected edge
      void singleEdgeCut(QMouseEvent* _event);
      // Select edges to be cut
      void selectEdge(QMouseEvent* _event);
      // Draw line to be cut
      void mouseDraw(QMouseEvent* _event);
      // Select vertices for splitting or merging
      void selectVertex(QMouseEvent* _event);
      // Solve and update positions of vertices on mesh
      void moveMesh(QMouseEvent* _event);
      // Show drawn path
      void showPath(ACG::Vec3d _prev_point, ACG::Vec3d _curr_point, BaseObjectData *object);
      // Finalize mouse interaction
      void applyCurve(BaseObjectData* object);
      bool updateMesh(bool _specify_n_iterations = false);

      QToolBar* toolBar_;
      bool mouseDown_;

      /// Cutting tools
      QAction* edgeCutAction_;
      QWidget* toolBox_;
      QPushButton* selectEdgesButton_;
      QPushButton* drawButton_;
      QCheckBox* clampToEdgeCheckBox_;
      QCheckBox* directCutCheckBox_;
      QCheckBox* independantCutsCheckBox_;
      QPushButton* selectVerticesButton_;

      /// Shape tools
      // Edge strain
      QCheckBox* edgeStrainCheckBox_;
      QDoubleSpinBox* edgeStrainWeightSpinBox_;
      // Triangle strain
      QCheckBox* triangleStrainCheckBox_;
      QDoubleSpinBox* triangleStrainWeightSpinBox_;
      // Area constraints
      QCheckBox* areaConstraintCheckBox_;
      QDoubleSpinBox* areaConstraintMinSpinBox_;
      QDoubleSpinBox* areaConstraintMaxSpinBox_;
      QDoubleSpinBox* areaConstraintWeightSpinBox_;
      // Bending constraints
      QCheckBox* bendingConstraintCheckBox_;
      QDoubleSpinBox* bendingConstraintMinSpinBox_;
      QDoubleSpinBox* bendingConstraintMaxSpinBox_;
      QDoubleSpinBox* bendingConstraintWeightSpinBox_;
      // Rectangle constraints
      QCheckBox* rectConstraintCheckBox_;
      QDoubleSpinBox* rectConstraintWeightSpinBox_;
      // Angle constraints
      QCheckBox* angleConstraintCheckBox_;
      QSpinBox* angleConstraintMinSpinBox_;
      QSpinBox* angleConstraintMaxSpinBox_;
      QDoubleSpinBox* angleConstraintWeightSpinBox_;
      // Solver
      QSpinBox* nSolverIterationsSpinBox_;

      /// Mesh generator
      // Quad
      QSpinBox* quadGenWidthSpinBox_;
      QSpinBox* quadGenHeightSpinBox_;
      // Tri
      QSpinBox* triGenRadiusSpinBox_;
      // Tessellation
      QCheckBox* hingedTessellationCheckBox_;

      Cutting* cutting_tools_;

      // Active elements set by setActiveElements()
      // Use mesh.[face|edge|vertex]_handle(idx) to access
      ACG::Vec3d active_hit_point_;
      int active_face_;
      int active_edge_;
      int active_vertex_;

      // Path drawn with the mouse
      std::vector<ACG::SceneGraph::LineNode*> visible_path_;

      // Record of last valid object
      BaseObjectData* latest_object_;

      ShapeTools* shape_tools_;
      bool use_shape_tools_;
      bool object_updated_;

      MeshTools* mesh_tools_;

   public:
      MeshCut() :
         toolBar_(0), mouseDown_(false), edgeCutAction_(0), toolBox_(0), selectEdgesButton_(), drawButton_(),
         clampToEdgeCheckBox_(), directCutCheckBox_(), independantCutsCheckBox_(), selectVerticesButton_(),
         edgeStrainCheckBox_(), edgeStrainWeightSpinBox_(),
         triangleStrainCheckBox_(), triangleStrainWeightSpinBox_(),
         areaConstraintCheckBox_(), areaConstraintMinSpinBox_(), areaConstraintMaxSpinBox_(), areaConstraintWeightSpinBox_(),
         bendingConstraintCheckBox_(), bendingConstraintMinSpinBox_(), bendingConstraintMaxSpinBox_(), bendingConstraintWeightSpinBox_(),
         rectConstraintCheckBox_(), rectConstraintWeightSpinBox_(),
         angleConstraintCheckBox_(), angleConstraintMinSpinBox_(), angleConstraintMaxSpinBox_(), angleConstraintWeightSpinBox_(),
         nSolverIterationsSpinBox_(),
         quadGenWidthSpinBox_(), quadGenHeightSpinBox_(), triGenRadiusSpinBox_(), hingedTessellationCheckBox_(),
         cutting_tools_(),
         active_hit_point_(0.0), active_face_(-1), active_edge_(-1), active_vertex_(-1),
         visible_path_(), latest_object_(0),
         shape_tools_(), use_shape_tools_(false), object_updated_(true),
         mesh_tools_() {}
      ~MeshCut(){ delete cutting_tools_; delete shape_tools_; delete mesh_tools_; }

      QString name() { return QString("MeshCut"); }
      QString description() { return QString("Cuts a mesh"); }
};


#endif // MESHCUT_HH
