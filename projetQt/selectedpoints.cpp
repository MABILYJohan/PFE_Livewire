#include "selectedpoints.h"

SelectedPoints::SelectedPoints()
{

}

void SelectedPoints::add_point(float x, float y)
{
    on_screen_points.append(MyMesh::Point(x, y, 0));
}

MyMesh::Point SelectedPoints::raycast_on_screen_point(float x, float y)
{

}

void SelectedPoints::raycast_selected_points()
{

}
