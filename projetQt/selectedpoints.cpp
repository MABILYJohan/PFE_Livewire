#include "selectedpoints.h"

SelectedPoints::SelectedPoints()
{

}

SelectedPoints::SelectedPoints(MyMesh *mesh)
{
    _mesh = mesh;
}


void SelectedPoints::add_point(float x, float y)
{
    on_screen_points.append(MyMesh::Point(x, y, 0));
}

MyMesh::Point SelectedPoints::raycast_on_screen_point(float x, float y)
{
    QVector3D intersection_p, raydir;


        Raycasting::disable_backface_culling();
        //if(Raycasting::intersects_triangle(screen_pov, raydir, fv, intersection_p))
        {

        }
        Raycasting::enable_backface_culling();

}

void SelectedPoints::raycast_selected_points()
{
    QVector3D intersection_p, raydir;
    MyMesh::Point nearest_vert;

    for(auto f_iter = _mesh->faces_begin(); f_iter != _mesh->faces_end(); f_iter++)
    {
        QVector<QVector3D> fv;
        for(auto fv_it = _mesh->fv_begin(*f_iter) ; fv_it.is_valid() ; fv_it++)
        {
            fv.append(QVector3D(_mesh->point(*fv_it)[0], _mesh->point(*fv_it)[1], _mesh->point(*fv_it)[2]));
        }

        for(auto p_iter = on_screen_points.begin(); p_iter != on_screen_points.end() ; p_iter++)
        {
            Raycasting::disable_backface_culling();
            bool intersection = Raycasting::intersects_triangle(screen_pov, raydir, fv, intersection_p);

            if(intersection)
            {
                double min;
                bool init_min = false;

                for(auto face_vertices = fv.begin() ; face_vertices != fv.end() ; face_vertices++)
                {
                    if(!init_min)
                    {
                        min = face_vertices->distanceToPoint(intersection_p);
                        nearest_vert[0] = face_vertices->x();
                        nearest_vert[1] = face_vertices->y();
                        nearest_vert[2] = face_vertices->z();
                        init_min = true;
                    }

                    else if(face_vertices->distanceToPoint(intersection_p) < min)
                    {
                        min = face_vertices->distanceToPoint(intersection_p);
                        nearest_vert[0] = face_vertices->x();
                        nearest_vert[1] = face_vertices->y();
                        nearest_vert[2] = face_vertices->z();
                    }
                }

                on_mesh_points.append(nearest_vert);
            }
        }
    }

}
