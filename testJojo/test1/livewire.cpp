#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    ;

    qDebug() << "</" << __FUNCTION__ << ">";
}
