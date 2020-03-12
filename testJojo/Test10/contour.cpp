#include "contour.h"


/////////////////////////// CONSTRUCTEURS   //////////////////////////////////////

Contour::Contour(MyMesh &_mesh) :
    mesh(_mesh)
{
    startPoint=-1;
    endPoint=-1;
}

Contour::Contour(MyMesh &_mesh, unsigned _begin) :
    mesh(_mesh)
{
    add_vertex(_begin);
}

Contour::Contour(MyMesh &_mesh, vector<unsigned> _vertices) :
    mesh(_mesh)
{
    verticesContour.clear();
    for (auto v : _vertices) {
        add_vertex(v);
    }
}

/*-------------------------------------------------------------
 * Cherche le sommet dont l'indice est dans @tmp,
 * qui est le plus proche du sommet d'indice @id.
 * Le sommet dans @tmp n'est reconnu que s'il n'est pas dans
 * @verticesContour.
 * Retourne -1 si aucun sommet compatible.
 * ----------------------------------------------------------*/
int Contour::search_min_dist_vertex_from_vertex(vector<int> tmp, int id)
{
    float min = static_cast<double>(INT_MAX);
    int idRes = -1;
    VertexHandle vh = mesh.vertex_handle(id);
    MyMesh::Point p = mesh.point(vh);

    for (auto t : tmp)
    {
        // Si sommet déjà dans le contour on saute
        if (Utils::is_in_vector(verticesContour, static_cast<unsigned>(t))) continue;

        VertexHandle vhTmp = mesh.vertex_handle(t);
        MyMesh::Point pTmp = mesh.point(vhTmp);
        pTmp = pTmp - p;
        float dist = pTmp.length();
        if (min > dist) {
            min = dist;
            idRes = t;
        }
    }

    return idRes;
}

/*---------------------------------------------------------------------
 * Cherche le point dans @tmp qui a, en fonction du boolean @b_max,
 * la @dim maximale ou minimale
 * ------------------------------------------------------------------*/
int Contour::search_borne_dim(vector<int> tmp, int dim, bool b_max)
{
    int idRes=-1;
    float borne;
    if (dim<0)  dim=0;
    if (dim>2)  dim=2;

    if (b_max)  borne = -INT_MAX;
    else        borne = INT_MAX;

    for (auto t : tmp)
    {
        // Si sommet déjà dans le contour on saute
        if (Utils::is_in_vector(verticesContour, static_cast<unsigned>(t))) continue;

        VertexHandle vhTmp = mesh.vertex_handle(t);
        MyMesh::Point pTmp = mesh.point(vhTmp);

        if (b_max)
        {
            if (borne < pTmp[dim]) {
                borne = pTmp[dim];
                idRes = t;
            }
        }
        else
        {
            if (borne > pTmp[dim]) {
                borne = pTmp[dim];
                idRes = t;
            }
        }
    }

    return idRes;
}

/*---------------------------------------------------------------------
 * Pour charger un contour à partir d'un maillage / nuage de points
 * ------------------------------------------------------------------*/
Contour::Contour(MyMesh &_mesh, char *path) :
    mesh(_mesh)
{
    qDebug() << "\t<" << __FUNCTION__ << ">";

    MyMesh myMeshContour;
    OpenMesh::IO::read_mesh(myMeshContour, path);
    vector<int> tmp;
    verticesContour.clear();

    // Appariement des points aux sommets les plus proches
    for (MyMesh::VertexIter curVert = myMeshContour.vertices_begin(); curVert != myMeshContour.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        MyMesh::Point P = myMeshContour.point(vh);
        int numVertex = UtilsMesh::find_near_vertex_of_point(&mesh, P);
        if ( ! Utils::is_in_vector(this->verticesContour, static_cast<unsigned>(numVertex))) {
            //this->add_vertex(numVertex);
            tmp.push_back(numVertex);
        }
    }
    Utils::suppr_occur(tmp);

    qDebug() << "\t\tnb de sommets contour = " << tmp.size();

    /////////////// V1 ////////////////
    //        int id=-1;
    //        if (tmp.size()>=4)
    //        {
    //             add_vertex(search_borne_dim(tmp, 0, false));
    //             add_vertex(search_borne_dim(tmp, 1, true));
    //             add_vertex(search_borne_dim(tmp, 0, true));
    //             add_vertex(search_borne_dim(tmp, 1, false));
    //        }

    /////////////// V2 ////////////////
    int id = 0;
    for (unsigned i=0; i<tmp.size()  ; i++)
    {
        //        qDebug() << "\t\ti=" << i;
        if (i==0) {
            this->add_vertex(tmp[i]);
            id = tmp[i];
        }
        else
        {
            id = search_min_dist_vertex_from_vertex(tmp, id);
            if (id<0) {
                qWarning() << "in" << __FUNCTION__ << ": id < 0";
                exit (2);
            }
            add_vertex(id);
        }
    }

    qDebug() << "\t</" << __FUNCTION__ << ">";
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return verticesContour; }
void Contour::set_contour(vector<unsigned> tmp)
{
    verticesContour.clear();
    for (auto v : tmp) {
        add_vertex(v);
    }
}


///////////////////////////// AUTRES   ////////////////////////////////////////

void Contour::display(int profDisplay, bool flagColor)
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
    int cpt=0;
    for (auto p : verticesContour)
    {
        qDebug() << cprof << "\tsommet =" << p;
        if (flagColor) {
            VertexHandle vh = mesh.vertex_handle(p);
            mesh.data(vh).thickness = 10;
            mesh.set_color(vh, MyMesh::Color(cpt, 255 - (4*cpt), 0));
            cpt++;
        }
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

    //    this->display(1);

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    int curVertex2 = verticesContour[cpt+1];

    qDebug() << "\t\tchargement draw: INIT ->" << verticesContour.size() << "sommets à calculer";
    LiveWire lW(*_mesh, curVertex, _sightPoint);

    if (curVertex2==endPoint && curVertex2!=startPoint)
    {
        //        qDebug() << "\t\tpremier if";
        qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
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
            //            qDebug() << "\t\tboucle cpt=" << cpt;
            qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
            curVertex2 = verticesContour[cpt+1];
            lW.update_vertexSeed(curVertex, curVertex2);
            lW.draw(curVertex2);
            curVertex = curVertex2;
            cpt++;
        }
    }


    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    lW.update_vertexSeed(endPoint, startPoint, true);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement draw:" << verticesContour.size()<<"/"<<verticesContour.size();

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



