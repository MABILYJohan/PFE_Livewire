#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    build();

    qDebug() << "</" << __FUNCTION__ << ">";
}

vector<unsigned> LiveWire::get_paths()  {   return paths;   }


double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    double cost = 0.0;
    EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);


    // Length
    cost = mesh.calc_edge_length(ehNeigh);
    // dihedral angle
    cost *= mesh.calc_dihedral_angle(ehNeigh);

    MyMesh::Point myNorm;
    myNorm = mesh.calc_edge_vector(ehNeigh);

    return cost;
}

void LiveWire::build()
{
    unsigned edgeBegin = 0;

    // Init
    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeBegin] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<unsigned> activeList;   activeList.push_back(edgeBegin);
    paths = vector<unsigned>(mesh.n_edges(), static_cast<unsigned>(-1));;

    while (!activeList.empty())
    {
        unsigned curEdge = Utils::get_min(activeList);
        Utils::erase_elt(activeList, curEdge);
        edgesVisited[curEdge] = true;

        // Voisinage
        vector<EdgeHandle> ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            unsigned edgeNeigh = eh.idx();
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
                activeList.push_back(edgeNeigh);
            }
        }
    }
}



// Brouillon
/*
 //    for (MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++)
 //    {
 //        EdgeHandle eh = *e_it;
 //        edges.push_back(eh.idx());
 //    }
*/













