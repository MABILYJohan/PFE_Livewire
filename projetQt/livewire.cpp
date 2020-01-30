#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    build();

    qDebug() << "</" << __FUNCTION__ << ">";
}

double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    double cost = 0.0;
    EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // TODO

    return cost;
}

void LiveWire::build()
{
    unsigned edgeBegin = 2;

    // Init
    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeBegin] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<unsigned> activeList;   activeList.push_back(edgeBegin);

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

                // TODO Indiquer le passage du curent au voisin comme passage

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













