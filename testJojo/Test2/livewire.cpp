#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    vector<int> tmp = {2, 5, 1};
    unsigned id;
    int val = Utils::get_min(tmp, id);
    qDebug() << "\tval" << id << " =" << val;

    qDebug() << "</" << __FUNCTION__ << ">";
}
