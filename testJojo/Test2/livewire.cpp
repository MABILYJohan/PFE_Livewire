#include "livewire.h"

#include <QDebug>

LiveWire::LiveWire(MyMesh &_mesh) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    build();

    qDebug() << "</" << __FUNCTION__ << ">";
}

void LiveWire::build()
{
    unsigned edgeBegin = 2;
    int totalCost = 0;
    vector<unsigned> edges;

    // Init
    edges.push_back(edgeBegin);

    while (!edges.empty())
    {
        unsigned curEdge = Utils::get_min(edges);
        Utils::erase_elt(edges, curEdge);
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













