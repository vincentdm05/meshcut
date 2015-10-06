#ifndef MESHCUT_HH
#define MESHCUT_HH

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/MouseInterface.hh>
// #include <OpenFlipper/BasePlugin/ToolboxInterface.hh>    // Uncomment all when adding toolbox
#include <OpenFlipper/BasePlugin/ToolbarInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/PickingInterface.hh>
#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>

class MeshCut : public QObject, BaseInterface, MouseInterface, /*ToolboxInterface,*/ ToolbarInterface, LoggingInterface, PickingInterface {
   Q_OBJECT
   Q_INTERFACES(BaseInterface)
   Q_INTERFACES(MouseInterface)
   // Q_INTERFACES(ToolboxInterface)
   Q_INTERFACES(ToolbarInterface)
   Q_INTERFACES(LoggingInterface)
   Q_INTERFACES(PickingInterface)

   signals:
      // BaseInterface
      void updateView();
      void updatedObject(int _id, const UpdateType& _type);

      // ToolboxInterface
      // void addToolbox( QString _name, QWidget* _widget );

      // ToolbarInterface
      void addToolbar(QToolBar* _toolbar);

      // LoggingInterface
      void log(Logtype _type, QString _message);
      void log(QString _message);

      // PickingInterface
      void addHiddenPickMode( const std::string& _mode );

   private slots:
      // BaseInterface
      void pluginsInitialized();

      /// ***** Toolbar *****
      // PickingInterface
      void slotPickModeChanged( const std::string& _mode);

      // MouseInterface
      void slotMouseEvent(QMouseEvent* _event);

      // Called when an action on the toolbar was triggered
      void toolBarTriggered(QAction* _action);
      /// *******************

      /// ***** Toolbox *****
      // void simpleCut();
      /// *******************

   public slots:
      QString version() { return QString("1.0"); };

   private:
      // Find selected edge
      void edgeCut(QMouseEvent* _event);
      // Cut along a single edge
      void cutPrimitive(TriMesh::EdgeHandle _edge, TriMesh& _mesh);

      QToolBar* toolBar_;
      QAction* edgeCutAction_;

   public:
      MeshCut();
      ~MeshCut(){};

      QString name() { return QString("MeshCut"); }
      QString description() { return QString("Cuts a mesh"); }
};


#endif // MESHCUT_HH
