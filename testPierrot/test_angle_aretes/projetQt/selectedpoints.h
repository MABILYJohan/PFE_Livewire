#ifndef SELECTEDPOINTS_H
#define SELECTEDPOINTS_H

#include <QVector>

#include "utils.h"
#include "utilsMesh.h"
#include "raycasting.h"

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class SelectedPoints
{
public:
    SelectedPoints();
    SelectedPoints(MyMesh *mesh);

    void add_point(float x, float y);
    void raycast_selected_points();

    QVector3D screen_pov; //Coordinates of the point of view

    QVector<MyMesh::Point> on_screen_points; //Store selected point on screen
    QVector<MyMesh::Point> on_mesh_points; //Store localisation of the points after raycasting
    MyMesh *_mesh;

private:
    MyMesh::Point raycast_on_screen_point(float x, float y); //Projection of the point (x,y) on the screen to the mesh (x',y',z')

};

#endif // SELECTEDPOINTS_H
