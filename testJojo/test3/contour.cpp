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

void Contour::build()
{
    qDebug() << "\t<" << __FUNCTION__ << ">";
    if (verticesContour.empty())    return;

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    unsigned curVertex2 = verticesContour[cpt+1];

    while (static_cast<int>(curVertex2) != endPoint)
    {
        curVertex2 = verticesContour[cpt+1];
    }

    qDebug() << "\t</" << __FUNCTION__ << ">";

}

void Contour::draw(MyMesh *_mesh)
{
    qDebug() << "\t<" << __FUNCTION__ << ">";
    if (edgesContour.empty())    return;

    unsigned curEdge = startPoint;
    unsigned cpt=0;
    unsigned curEdge2 = edgesContour[cpt+1];


    while (static_cast<int>(curEdge2) != endPoint)
    {
        qDebug() << "\t\tchargement:" << cpt<<"/"<<edgesContour.size();
        //        qDebug() << "\t\tcpt=" << cpt;
        curEdge2 = edgesContour[cpt+1];
        //        qDebug() << "\t\tcurEdge=" << curEdge;
        //        qDebug() << "\t\tcurEdge2=" << curEdge2;

        LiveWire lW(*_mesh, curEdge);
        lW.draw(curEdge2);
        curEdge = curEdge2;
        cpt++;
    }
    qDebug() << "\t\tchargement:" << cpt<<"/"<<edgesContour.size();
    LiveWire lW(*_mesh, endPoint);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement:" << cpt+1<<"/"<<edgesContour.size();
    qDebug() << "\t</" << __FUNCTION__ << ">";
}














