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
      void cutPrimitive(TriMesh::EdgeHandle edge, TriMesh& mesh);
      // Connect adjacent cuts
      void connectCuts(TriMesh::VertexHandle vh, TriMesh &mesh);

      QToolBar* toolBar_;
      QAction* edgeCutAction_;
      QWidget* toolBox_;
      QPushButton* selectButton_;
      QPushButton* drawButton_;
      // 0 none toggled, 1 select edges, 2 draw line
      size_t selectionButtonToggled_;

      // Active TriMesh edge handle and mesh, for handling purpose
      // Use mesh.[edge|vertex]_handle(int) to access
      unsigned int active_edge_;
      unsigned int active_vertex_;
      ACG::Vec3d active_edge_point_;

   public:
      MeshCut();
      ~MeshCut(){}

      QString name() { return QString("MeshCut"); }
      QString description() { return QString("Cuts a mesh"); }
};


#endif // MESHCUT_HH
