#ifndef SELECTEDPOINTS_H
#define SELECTEDPOINTS_H

#include <QVector>

#include "utils.h"
#include "utilsMesh.h"

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class SelectedPoints
{
public:
    SelectedPoints();

    MyMesh::Point raycast_onscreen_point(float x, float y); //Localization of the point (x,y) on the screen in the 3D mesh (x',y',z')
    QVector<MyMesh::Point> points; //Store localisation of the point after raycasting

};

#endif // SELECTEDPOINTS_H
