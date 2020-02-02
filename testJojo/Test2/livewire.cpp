#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh, int _edgeSeed,  MyMesh::Point _sightPoint) :
    mesh(_mesh), edgeSeed(_edgeSeed), sightPoint(_sightPoint)
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
    double cost = 0.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // Length
    cost = mesh.calc_edge_length(ehNeigh);
    // dihedral angle
    cost *= mesh.calc_dihedral_angle(ehNeigh);
    // Normal orientation
    cost*=normal_orientation(numEdgeNeigh, sightPoint);

    // TESTS
    MyMesh::Point myNorm;
    myNorm = mesh.calc_edge_vector(ehNeigh);
    //    mesh.has_halfedge_normals ();
    // const Normal & 	normal (HalfedgeHandle _heh) const

    return cost;
}

void LiveWire::build()
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    // Init
    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeSeed] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;   activeList.push_back(edgeSeed);
    paths = vector<int>(mesh.n_edges(), -1);

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS
    //    unsigned cpt=0;
    while (!activeList.empty())
    {
        //        qDebug() << "\t\t\t" << cpt++ ;
        //        qDebug() << activeList;
        int curEdge = Utils::get_min(activeList);
        Utils::erase_elt(activeList, curEdge);
        edgesVisited[curEdge] = true;

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
//    // TMP anti bug
//    if (edgeBegin!=mesh.n_edges()-1)
//        paths[edgeBegin] = edgeBegin+1;

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned edge2)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    EdgeHandle eh1 = mesh.edge_handle(edgeSeed);
    EdgeHandle eh2 = mesh.edge_handle(edge2);

    unsigned curEdge = edge2;
    //    qDebug() << "\t\t\tedgeSeed=" << edgeSeed;
    //    qDebug() << "\t\t\tedge2=" << edge2;

    vector<EdgeHandle> ehs;

    //    qDebug() << "\t\t\tcurEdge=" << curEdge;
    while (static_cast<int>(curEdge) != edgeSeed)
    {
        ehs.clear();
        ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            if (eh.idx()==paths[curEdge]) {
                mesh.set_color(eh, MyMesh::Color(0, 0, 255));
                mesh.data(eh).thickness = 6;
            }
        }
        if (ehs.empty())    break;
        curEdge = paths[curEdge];
        //        qDebug() << "\t\t\tcurEdge=" << curEdge;
    }

    // point de départ et point d'arrivée en vert et en gros
    mesh.set_color(eh1, MyMesh::Color(0, 255, 0));
    mesh.set_color(eh2, MyMesh::Color(0, 255, 0));
    mesh.data(eh1).thickness = 8;
    mesh.data(eh2).thickness = 8;

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}













