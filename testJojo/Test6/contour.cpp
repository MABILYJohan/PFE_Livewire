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
    verticesContour.clear();
    for (auto v : _vertices) {
        add_vertex(v);
    }
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return verticesContour; }

void Contour::display(int profDisplay)
{
    if (profDisplay<0)  profDisplay=0;
    if (profDisplay>5)  profDisplay=5;
    char prof[profDisplay+1];
    for (int i=0; i< profDisplay; i++) {
        prof[i] = '\t';
    }
    prof[profDisplay] = '\0';
    char *cprof = prof;

    qDebug() << cprof << "<" << __FUNCTION__ << ">";

    qDebug() << cprof << "\tstart =" << startPoint;
    for (auto p : verticesContour)
    {
        qDebug() << cprof << "\tpoint =" << p;
    }
    qDebug() << cprof << "\tend =" << endPoint;

    qDebug() << cprof << "</" << __FUNCTION__ << ">";
}

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

    this->display(1);

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    unsigned curVertex2 = verticesContour[cpt+1];

    //    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    LiveWire lW(*_mesh, curVertex, _sightPoint);

    if (curVertex2==endPoint && curVertex2!=startPoint)
    {
        //        qDebug() << "\t\tpremier if";
        //        qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
        curVertex2 = verticesContour[cpt+1];
        lW.update_vertexSeed(curVertex, curVertex2);
        lW.draw(curVertex2);
        curVertex = curVertex2;
        cpt++;
    }
    else
    {
        while (static_cast<int>(curVertex2) != endPoint)
        {
            qDebug() << "\t\tboucle cpt=" << cpt;
            //            qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
            curVertex2 = verticesContour[cpt+1];
            lW.update_vertexSeed(curVertex, curVertex2);
            lW.draw(curVertex2);
            curVertex = curVertex2;
            cpt++;
        }
    }


    //    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    lW.update_vertexSeed(endPoint, startPoint);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement draw:" << verticesContour.size()<<"/"<<verticesContour.size();

    qDebug() << "\t</" << __FUNCTION__ << ">";
}









