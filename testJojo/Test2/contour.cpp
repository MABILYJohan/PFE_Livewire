#include "contour.h"


Contour::Contour()
{
    ;
}

Contour::Contour(unsigned _begin)
{
    add_edge(_begin);
}

Contour::Contour(vector<unsigned> _edges)
{
    for (auto e : _edges) {
        add_edge(e);
    }
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return contour; }

void Contour::add_edge(unsigned numEdge)
{
    if (contour.empty())
        startPoint = numEdge;

    contour.push_back(numEdge);
    endPoint = numEdge;
}
