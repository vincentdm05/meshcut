#ifndef MESHCUT
#define MESHCUT

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/common/Types.hh>

class MeshCut : public QObject, BaseInterface {
    Q_OBJECT
    Q_INTERFACES(BaseInterface)

public:
    ~MeshCut(){}

    QString name() { return QString("MESHCUT"); }
    QString description() { return QString("Does actually nothing but works!"); }
};


#endif // MESHCUT

