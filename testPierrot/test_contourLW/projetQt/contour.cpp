#include "contour.h"


Contour::Contour()
{
    startPoint=-1;
    endPoint=-1;
}

Contour::Contour(unsigned _begin)
{
    add_vertex(_begin);
}

Contour::Contour(vector<unsigned> _vertices)
{
    verticesContour.clear();
    for (auto v : _vertices) {
        add_vertex(v);
    }
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return verticesContour; }

void Contour::display(int profDisplay)
{
    if (profDisplay<0)  profDisplay=0;
    if (profDisplay>5)  profDisplay=5;
    char prof[profDisplay+1];
    for (int i=0; i< profDisplay; i++) {
        prof[i] = '\t';
    }
    prof[profDisplay] = '\0';
    char *cprof = prof;

    qDebug() << cprof << "<" << __FUNCTION__ << ">";

    qDebug() << cprof << "\tstart =" << startPoint;
    for (auto p : verticesContour)
    {
        qDebug() << cprof << "\tpoint =" << p;
    }
    qDebug() << cprof << "\tend =" << endPoint;

    qDebug() << cprof << "</" << __FUNCTION__ << ">";
}

/* Attention obsolète: à remanier.*/
void Contour::add_edge(unsigned numEdge)
{
    if (edgesContour.empty())
        startPoint = numEdge;

    edgesContour.push_back(numEdge);
    endPoint = numEdge;
}

void Contour::add_vertex(unsigned numVertex)
{
    if (verticesContour.empty())
        startPoint = numVertex;

    verticesContour.push_back(numVertex);
    endPoint = numVertex;
}

void Contour::draw_contour(MyMesh *_mesh, MyMesh::Point _sightPoint)
{
    qDebug() << "\t<" << __FUNCTION__ << ">";
    if (verticesContour.empty())    return;

    this->display(1);

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    unsigned curVertex2 = verticesContour[cpt+1];

    //    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    LiveWire lW(*_mesh, curVertex, _sightPoint);

    if (curVertex2==endPoint && curVertex2!=startPoint)
    {
        //        qDebug() << "\t\tpremier if";
        //        qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
        curVertex2 = verticesContour[cpt+1];
        lW.update_vertexSeed(curVertex, curVertex2);
        lW.draw(curVertex2);
        curVertex = curVertex2;
        cpt++;
    }
    else
    {
        while (static_cast<int>(curVertex2) != endPoint)
        {
            qDebug() << "\t\tboucle cpt=" << cpt;
            //            qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
            curVertex2 = verticesContour[cpt+1];
            lW.update_vertexSeed(curVertex, curVertex2);
            lW.draw(curVertex2);
            curVertex = curVertex2;
            cpt++;
        }
    }


    //    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    lW.update_vertexSeed(endPoint, startPoint);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement draw:" << verticesContour.size()<<"/"<<verticesContour.size();

    this->edgesLW = lW.get_edgesLW();
    for (auto e : edgesLW) {
        VertexHandle vh0, vh1;
        UtilsMesh::get_vh_of_edge(_mesh, e, vh0, vh1);
        if (Utils::distance_euclidienne(_mesh->point(vh0)[0], _mesh->point(vh1)[0], _mesh->point(vh0)[1],
                                        _mesh->point(vh1)[1], _mesh->point(vh0)[2], _mesh->point(vh1)[2])<0.5)
        {
          qDebug() << "edge : " << e;
          //UtilsMesh::splitEdge(_mesh, e, true);
          //UtilsMesh::collapseEdge(_mesh, e);
        }
    }

    qDebug() << "\t</" << __FUNCTION__ << ">";
}

#if 0

void SelectedPoints::add_point(float x, float y)
{
    on_screen_points.append(MyMesh::Point(x, y, 0));
}

MyMesh::Point Contour::raycast_on_screen_point(float x, float y) //TODO
{
    QVector3D intersection_p, raydir;


        Raycasting::disable_backface_culling();
        //if(Raycasting::intersects_triangle(screen_pov, raydir, fv, intersection_p))
        {

        }
        Raycasting::enable_backface_culling();

}

//todo correction
void Contour::raycast_selected_points()
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

                //on_mesh_points.append(nearest_vert);
            }
        }
    }

}

#endif



