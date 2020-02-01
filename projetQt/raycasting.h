#ifndef RAYCASTING_H
#define RAYCASTING_H
#include <QVector3D>

namespace Raycasting
{
    inline QVector3D parametric_point_on_line(QVector3D P, QVector3D O);
    inline QVector3D cross(QVector3D P, QVector3D O);
    inline QVector3D dot(QVector3D P, QVector3D O);

}

#endif // RAYCASTING_H
