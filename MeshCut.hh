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
      BaseObjectData *setActiveElements(QMouseEvent* _event);
      // Select edges to be cut
      void selectEdge(QMouseEvent* _event);
      // Draw line to be cut
      void drawLine(QMouseEvent* _event);
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
      int active_face_;
      int active_edge_;
      int active_vertex_;  /// Do we need that?
      ACG::Vec3d active_hit_point_;

      // Record of last valid object
      BaseObjectData* latest_object_;
      int latest_edge_;

      // Previously active components
      int prev_face_;
      int prev_edge_;
      ACG::Vec3d prev_hit_point_;

      // Edge crossings
      ACG::Vec3d active_edge_crossing_;
      ACG::Vec3d prev_edge_crossing_;

      // Vertices at splits
      int active_vertex_at_split_;
      int prev_vertex_at_split_;

   public:
      MeshCut();
      ~MeshCut(){}

      QString name() { return QString("MeshCut"); }
      QString description() { return QString("Cuts a mesh"); }
};


#endif // MESHCUT_HH
