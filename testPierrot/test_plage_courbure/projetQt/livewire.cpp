#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint) :
    mesh(_mesh), vertexSeed(_vertexSeed), sightPoint(_sightPoint)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    build();

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

vector<int> LiveWire::get_paths()  {   return paths;   }

double LiveWire::normal_orientation(int numEdge, MyMesh::Point _sightPoint)
{
    double cost=0.0;

    EdgeHandle eh = mesh.edge_handle(numEdge);

    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);
    MyMesh::Point myVec = myP - _sightPoint;

    MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 0);
    FaceHandle fh1 = mesh.face_handle(heh1);
    MyMesh::Normal n1 = mesh.calc_face_normal(fh1);
    float angle1 = acos(dot(n1, myVec));
    cost+=angle1;

    if ( ! mesh.is_boundary(eh) )
    {
        MyMesh::HalfedgeHandle heh2 = mesh.halfedge_handle(eh, 1);
        FaceHandle fh2 = mesh.face_handle(heh2);
        MyMesh::Normal n2 = mesh.calc_face_normal(fh2);
        float angle2 = acos(dot(n2, myVec));
        cost+=angle2;
    }

    return cost;
}

/*---------------------------------------
 * Fonctions pour la courbure gaussienne
 * --------------------------------------*/

float LiveWire::angleEE(MyMesh* _mesh, int vertexID,  int faceID)
{
    MyMesh::Point v1;
    MyMesh::Point v2;
    FaceHandle fhId = _mesh->face_handle(faceID);
    VertexHandle vhId = _mesh->vertex_handle(vertexID);
    std::vector<VertexHandle> vh;
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fhId); fv_it.is_valid(); fv_it++)
    {
        vh.push_back(*fv_it);
    }

    MyMesh::Point A;
    MyMesh::Point B;
    MyMesh::Point C;
    for (unsigned i=0; i<vh.size(); i++)
    {
        if (vh[i] == vhId) {
            A = _mesh->point (vh[i]);
            int k=i+1;
            if (k>=vh.size()) k=0;
            B = _mesh->point (vh[k]);
            k++;
            if (k>=vh.size()) k=0;
            C = _mesh->point (vh[k]);
            break;
        }
    }
    v1 = B-A;
    v2 = C-A;
    v1.normalize();
    v2.normalize();

    float angle = acos((v1 | v2));

    return angle;
}

float LiveWire::faceArea(MyMesh* _mesh, int faceID)
{

    FaceHandle face_h = FaceHandle(faceID);
    QVector<MyMesh::Point> points;
    for(MyMesh::FaceVertexIter curVer = _mesh->fv_iter(face_h); curVer.is_valid(); curVer++) {
        VertexHandle vertex_h = *curVer;
        points.push_back(_mesh->point(vertex_h));
    }

    float aire = norm((points[1] - points[0]) % (points[2] - points[0])) / 2;
    return aire;
}

float LiveWire::aire_barycentrique(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float area = 0;
    for(MyMesh::VertexFaceIter vfit = _mesh->vf_iter(vh); vfit.is_valid(); vfit++){
        area += faceArea(_mesh,(*vfit).idx());
    }
    return area / 3;
}

double LiveWire::K_Curv(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float a = 1 / aire_barycentrique(_mesh, vh.idx());
    float theta = 0.f;
    for (MyMesh::VertexFaceCWIter vf_it = _mesh->vf_cwiter(vh); vf_it.is_valid(); vf_it++)
        {
            FaceHandle fh = *vf_it;
            theta += angleEE(_mesh, vh.idx(), fh.idx());
        }
    float b = 2*M_PI - theta;
    float K = a*b;
    qDebug() << K;
    return K;
}


double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    /*double moyenne, K;
    for (MyMesh::VertexIter curVert = mesh.vertices_begin(); curVert != mesh.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        K+=K_Curv(&mesh, vh.idx());
    }
    moyenne=K/mesh.n_vertices();
    qDebug()<<"moyenne :"<<moyenne;

    double et;
    for (MyMesh::VertexIter curVert = mesh.vertices_begin(); curVert != mesh.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        et+=((K_Curv(&mesh, vh.idx()))-moyenne)*((K_Curv(&mesh, vh.idx()))-moyenne);
    }
    et=sqrt(et/mesh.n_vertices());
    qDebug()<<"ecart type :"<<et;*/



    double cost = 0.0;
    //EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // Length
    cost = mesh.calc_edge_length(ehNeigh);
    // dihedral angle
    cost *= mesh.calc_dihedral_angle(ehNeigh);
    // Normal orientation
    cost*=normal_orientation(numEdgeNeigh, sightPoint);

    VertexHandle vh_commun, vh_suivant;
    UtilsMesh::get_vh_of_edge(&mesh, numEdgeNeigh, vh_commun, vh_suivant);
    double K_commun = K_Curv(&mesh, vh_commun.idx());
    double K_suivant = K_Curv(&mesh, vh_suivant.idx());
    //qDebug() << "K_commun = " << K_commun;
    //qDebug() << "K_suivant = " << K_suivant;
    cost*=(K_commun+K_suivant)/2.0;
    //    // TESTS
    //    MyMesh::Point myNorm;
    //    myNorm = mesh.calc_edge_vector(ehNeigh);
    //    //    mesh.has_halfedge_normals ();
    //    // const Normal & 	normal (HalfedgeHandle _heh) const

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
    return cost;
}

unsigned get_minCostEdge_from_activeList(vector<int> activeList, vector<double> costEdges)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    unsigned numEdge=0;
    double min = static_cast<double>(INT_MAX);
    for (auto e : activeList) {
        if (costEdges[e] <= min) {
            min = costEdges[e];
            numEdge = e;
        }
    }
    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
    return numEdge;
}

void LiveWire::build()
{
    //    qDebug() << "\t<" << __FUNCTION__ << ">";

    // Init
    EdgeHandle ehTmp = UtilsMesh::get_next_eh_of_vh(&mesh, vertexSeed);
    edgeSeed = ehTmp.idx();

    //    qDebug() << "\t\tvertexSeed =" << vertexSeed;
    //    qDebug() << "\t\tedgeSeed =" << edgeSeed;

    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeSeed] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;   activeList.push_back(edgeSeed);
    paths = vector<int>(mesh.n_edges(), -1);

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    while (!activeList.empty())
    {
        int curEdge = get_minCostEdge_from_activeList(activeList, costEdges);
        //        int curEdge = Utils::get_min(costEdges);
        Utils::erase_elt(activeList, curEdge);
        edgesVisited[curEdge] = true;

        //        qDebug() << "\t\tcurEdge =" << curEdge;
        //        qDebug() << activeList;
        // Voisinage
        vector<EdgeHandle> ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            int edgeNeigh = eh.idx();
            // Si déjà visité
            if (edgesVisited[edgeNeigh])    continue;

            double tmpCost = costEdges[curEdge] + cost_function(curEdge, edgeNeigh) ;

            // Voisin dans liste active ET  coût calculé inférieur au coût enregistré
            if (Utils::is_in_vector(activeList, edgeNeigh) &&  tmpCost < costEdges[edgeNeigh]) {
                Utils::erase_elt(activeList, edgeNeigh);
            }
            // Voisin pas dans la lsite active
            else if ( ! Utils::is_in_vector(activeList, edgeNeigh)) {
                costEdges[edgeNeigh] = tmpCost;
                 paths[edgeNeigh] = curEdge;
                // paths[curEdge] = edgeNeigh;
                activeList.push_back(edgeNeigh);
            }
        }
    }
    //    qDebug() << "\t</" << __FUNCTION__ << ">";
}

/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned vertex2)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";
    srand(0);
    int red = Utils::randInt(0, 255);
    int blue = Utils::randInt(0, 255);
    int green = Utils::randInt(0, 255);

    EdgeHandle eh1 = mesh.edge_handle(edgeSeed);
    //    EdgeHandle eh2 = mesh.edge_handle(vertex2);
    EdgeHandle eh2 = UtilsMesh::get_next_eh_of_vh(&mesh, vertex2);

    //    qDebug() << "\t\tTest";

    unsigned curEdge = vertex2;
    //    qDebug() << "\t\t\tedgeSeed=" << edgeSeed;
    //    qDebug() << "\t\t\tedge2=" << edge2;

    vector<EdgeHandle> ehs;

    //    qDebug() << "\t\t\tcurEdge=" << curEdge;
    while (static_cast<int>(curEdge) != edgeSeed)
    {
        //        qDebug() << "\t\tcurEdge =" << curEdge;

        ehs.clear();
        ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            if (eh.idx()==paths[curEdge]) {
                mesh.set_color(eh, MyMesh::Color(red, blue, green));
                mesh.data(eh).thickness = 6;
            }
        }
        if (ehs.empty())    break;
        curEdge = paths[curEdge];
        //        qDebug() << "\t\t\tcurEdge=" << curEdge;
    }

    // point de départ et point d'arrivée en rouge et en gros
    mesh.set_color(eh1, MyMesh::Color(255, 0, 0));
    mesh.set_color(eh2, MyMesh::Color(255, 0, 0));
    mesh.data(eh1).thickness = 8;
    mesh.data(eh2).thickness = 8;
    VertexHandle vh1 = mesh.vertex_handle(vertexSeed);
    VertexHandle vh2 = mesh.vertex_handle(vertex2);
    mesh.set_color(vh1, MyMesh::Color(255, 0, 0));
    mesh.set_color(vh2, MyMesh::Color(255, 0, 0));
    mesh.data(vh1).thickness = 20;
    mesh.data(vh2).thickness = 20;

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}













