#include "contour.h"


Contour::Contour()
{
    startPoint=-1;
    endPoint=-1;
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

void Contour::draw(MyMesh *_mesh, MyMesh::Point _sightPoint)
{
    qDebug() << "\t<" << __FUNCTION__ << ">";
    if (contour.empty())    return;

    unsigned curEdge = startPoint;
    unsigned cpt=0;
    unsigned curEdge2 = contour[cpt+1];


    while (static_cast<int>(curEdge2) != endPoint)
    {
        qDebug() << "\t\tchargement:" << cpt<<"/"<<contour.size();
        //        qDebug() << "\t\tcpt=" << cpt;
        curEdge2 = contour[cpt+1];
        //        qDebug() << "\t\tcurEdge=" << curEdge;
        //        qDebug() << "\t\tcurEdge2=" << curEdge2;

        LiveWire lW(*_mesh, curEdge, _sightPoint);
        lW.draw(curEdge2);
        curEdge = curEdge2;
        cpt++;
    }
    qDebug() << "\t\tchargement:" << cpt<<"/"<<contour.size();
    LiveWire lW(*_mesh, endPoint, _sightPoint);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement:" << cpt+1<<"/"<<contour.size();
    qDebug() << "\t</" << __FUNCTION__ << ">";
}














