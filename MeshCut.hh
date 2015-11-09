#ifndef MESHCUT_HH
#define MESHCUT_HH

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/MouseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolbarInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/PickingInterface.hh>
#include <OpenFlipper/BasePlugin/BackupInterface.hh>
#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ACG/Scenegraph/LineNode.hh>
#include <eigen3/Eigen/Geometry>

#include <queue>
#include <tuple>

#define TUPLE_POINT 0
#define TUPLE_FACE 1
#define TUPLE_EDGE 2

class MeshCut : public QObject, BaseInterface, MouseInterface, ToolbarInterface, ToolboxInterface, LoggingInterface, PickingInterface, BackupInterface {
   Q_OBJECT
   Q_INTERFACES(BaseInterface)
   Q_INTERFACES(MouseInterface)
   Q_INTERFACES(ToolbarInterface)
   Q_INTERFACES(ToolboxInterface)
   Q_INTERFACES(LoggingInterface)
   Q_INTERFACES(PickingInterface)
   Q_INTERFACES(BackupInterface)

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

   private slots:
      // BaseInterface
      void initializePlugin();
      void pluginsInitialized();

      // PickingInterface
      void slotPickModeChanged( const std::string& _mode);

      // MouseInterface
      void slotMouseEvent(QMouseEvent* _event);

      // Called when an action on the toolbar was triggered
      void toolBarTriggered(QAction* _action);

      // Cut mode selection
      void slotSelectionButtonClicked();

      // Called when toolbox button is clicked
      void slotCutSelectedEdges();

   public slots:
      QString version() { return QString("1.0"); }

   private:
      // Sets active mesh, edge and vertex
      BaseObjectData *setActiveElements(QPoint _pos);

      // Select edges to be cut
      void selectEdge(QMouseEvent* _event);

      // Draw line to be cut
      void mouseDraw(QMouseEvent* _event);

      // Show drawn path
      void showPath(ACG::Vec3d _prev_point, ACG::Vec3d _curr_point, BaseObjectData *object);

      // Finalize mouse interaction
      void applyCurve(BaseObjectData* object);

      // Draw closest approximation of drawn curve on mesh
      void markForSplit(BaseObjectData *object);

      // Find if two segments intersect and set intersection_point if appropriate
      bool segmentIntersect(Eigen::Vector3d p0, Eigen::Vector3d p1,
                            Eigen::Vector3d q0, Eigen::Vector3d q1, Eigen::Vector3d* intersection_point);

      // Determine whether a point lies on a segment
      bool isOnSegment(Eigen::Vector3d p, Eigen::Vector3d s0, Eigen::Vector3d s1, double prec);

      // Split marked edges and select new applied path
      void splitAndSelect(TriMesh& mesh);
      void splitAndSelect(PolyMesh& mesh);

      // Find selected edge
      void singleEdgeCut(QMouseEvent* _event);

      // Cut along a single edge
      template<typename MeshT>
      bool cutPrimitive(typename MeshT::EdgeHandle edge, MeshT &mesh);

      // Connect adjacent cuts
      template<typename MeshT>
      void connectCuts(typename MeshT::VertexHandle vh, MeshT& mesh);

      // Get edge between two vertices
      template<typename MeshT>
      int edge_between(typename MeshT::VertexHandle vh_from,
                       typename MeshT::VertexHandle vh_to, MeshT& mesh);

      QToolBar* toolBar_;
      QAction* edgeCutAction_;
      QWidget* toolBox_;
      QPushButton* selectButton_;
      QPushButton* drawButton_;
      // 0 none toggled, 1 select edges, 2 draw line
      size_t selectionButtonToggled_;

      // Active elements set by setActiveElements()
      // Use mesh.[face|edge|vertex]_handle(idx) to access
      ACG::Vec3d active_hit_point_;
      int active_face_;
      int active_edge_;
      int active_vertex_;  /// TODO: Do we need that?

      // Path drawn with the mouse
      std::vector<ACG::SceneGraph::LineNode*> visible_path_;

      // Real path on the mesh: hit point, hit face, closest edge
      /// TODO: Do we need closest edge?
      std::list<std::tuple<ACG::Vec3d,int,int> > recorded_path_;

      // Queue of edge handle and crossing point pairs
      std::queue<std::pair<int,ACG::Vec3d> > edges_to_split_;

      // Record of last valid object
      BaseObjectData* latest_object_;

   public:
      MeshCut();
      ~MeshCut(){}

      QString name() { return QString("MeshCut"); }
      QString description() { return QString("Cuts a mesh"); }
};


#endif // MESHCUT_HH
