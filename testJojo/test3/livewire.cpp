#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh, int _vertexSeed) :
    mesh(_mesh), vertexSeed(_vertexSeed)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    build();

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

vector<int> LiveWire::get_paths()  {   return paths;   }


double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    double cost = 0.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // Length
    cost = mesh.calc_edge_length(ehNeigh);
    // dihedral angle
    cost *= mesh.calc_dihedral_angle(ehNeigh);

    // TESTS
    MyMesh::Point myNorm;
    myNorm = mesh.calc_edge_vector(ehNeigh);
    //    mesh.has_halfedge_normals ();
    // const Normal & 	normal (HalfedgeHandle _heh) const

    return cost;
}

unsigned get_minCostEdge_from_activeList(vector<int> activeList, vector<double> costEdges)
{
    unsigned numEdge=0;
    double min = static_cast<double>(INT_MAX);
    for (auto e : activeList) {
        if (costEdges[e] <= min) {
            min = costEdges[e];
            numEdge = e;
        }
    }
    return numEdge;
}

void LiveWire::build()
{
    // Init
    unsigned edgeSeed;
    VertexHandle vhTmp = mesh.vertex_handle(vertexSeed);
    for (MyMesh::VertexEdgeCWIter ve_it = mesh.ve_cwiter(vhTmp); ve_it.is_valid(); ve_it++)
    {
        EdgeHandle ehTmp = *ve_it;
        edgeSeed = ehTmp.idx();
        break;
    }

    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeSeed] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;   activeList.push_back(edgeSeed);
    paths = vector<int>(mesh.n_edges(), -1);

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    while (!activeList.empty())
    {
        unsigned curEdge = get_minCostEdge_from_activeList(activeList, costEdges);
        //        Utils::get_min(costEdges, curEdge);
        Utils::erase_elt(activeList, static_cast<int>(curEdge));
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
}

/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned vertex2)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    EdgeHandle eh1 = mesh.edge_handle(vertexSeed);
    EdgeHandle eh2 = mesh.edge_handle(vertex2);

    unsigned curEdge = vertex2;
    //    qDebug() << "\t\t\tedgeSeed=" << edgeSeed;
    //    qDebug() << "\t\t\tedge2=" << edge2;

    vector<EdgeHandle> ehs;

    //    qDebug() << "\t\t\tcurEdge=" << curEdge;
    while (static_cast<int>(curEdge) != vertexSeed)
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













