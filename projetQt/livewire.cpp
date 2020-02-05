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

double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    double cost = 0.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // Length
    cost = mesh.calc_edge_length(ehNeigh);
    // dihedral angle
    cost *= mesh.calc_dihedral_angle(ehNeigh);
    // Normal orientation
    cost*=normal_orientation(numEdgeNeigh, sightPoint);

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













