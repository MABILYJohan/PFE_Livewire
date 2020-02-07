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
    for (auto v : _vertices) {
        add_vertex(v);
    }
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return edgesContour; }

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

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    unsigned curVertex2 = verticesContour[cpt+1];

    LiveWire lW(*_mesh, _sightPoint);

    while (static_cast<int>(curVertex2) != endPoint)
    {
        qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
        //        qDebug() << "\t\tcpt=" << cpt;
        curVertex2 = verticesContour[cpt+1];
        //        qDebug() << "\t\tcurVertex=" << curEdge;
        //        qDebug() << "\t\tcurVertex2=" << curEdge2;

        //        LiveWire lW(*_mesh, curVertex, _sightPoint);
        lW.build_paths(curVertex);
        lW.draw(curVertex2);
        curVertex = curVertex2;
        cpt++;
    }
    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    lW.build_paths(endPoint);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement draw:" << cpt+1<<"/"<<verticesContour.size();
    qDebug() << "\t</" << __FUNCTION__ << ">";
}










