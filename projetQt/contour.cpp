#include "contour.h"


/////////////////////////// CONSTRUCTEURS   //////////////////////////////////////

Contour::Contour(MyMesh &_mesh) :
    mesh(_mesh)
{
    startPoint=-1;
    endPoint=-1;
}

Contour::Contour(MyMesh &_mesh, unsigned _begin) :
    mesh(_mesh)
{
    add_vertex(_begin);
}

Contour::Contour(MyMesh &_mesh, vector<unsigned> _vertices) :
    mesh(_mesh)
{
    verticesContour.clear();
    for (auto v : _vertices) {
        add_vertex(v);
    }
}

/*-------------------------------------------------------------
 * Cherche le sommet dont l'indice est dans @tmp,
 * qui est le plus proche du sommet d'indice @id.
 * Le sommet dans @tmp n'est reconnu que s'il n'est pas dans
 * @verticesContour.
 * Retourne -1 si aucun sommet compatible.
 * ----------------------------------------------------------*/
int Contour::search_min_dist_vertex_from_vertex(vector<int> tmp, int id)
{
    float min = static_cast<double>(INT_MAX);
    int idRes = -1;
    VertexHandle vh = mesh.vertex_handle(id);
    MyMesh::Point p = mesh.point(vh);

    for (auto t : tmp)
    {
        // Si sommet déjà dans le contour on saute
        if (Utils::is_in_vector(verticesContour, static_cast<unsigned>(t))) continue;

        VertexHandle vhTmp = mesh.vertex_handle(t);
        MyMesh::Point pTmp = mesh.point(vhTmp);
        pTmp = pTmp - p;
        float dist = pTmp.length();
        if (min > dist) {
            min = dist;
            idRes = t;
        }
    }

    return idRes;
}

/*---------------------------------------------------------------------
 * Cherche le point dans @tmp qui a, en fonction du boolean @b_max,
 * la @dim maximale ou minimale
 * ------------------------------------------------------------------*/
int Contour::search_borne_dim(vector<int> tmp, int dim, bool b_max)
{
    int idRes=-1;
    float borne;
    if (dim<0)  dim=0;
    if (dim>2)  dim=2;

    if (b_max)  borne = -INT_MAX;
    else        borne = INT_MAX;

    for (auto t : tmp)
    {
        // Si sommet déjà dans le contour on saute
        if (Utils::is_in_vector(verticesContour, static_cast<unsigned>(t))) continue;

        VertexHandle vhTmp = mesh.vertex_handle(t);
        MyMesh::Point pTmp = mesh.point(vhTmp);

        if (b_max)
        {
            if (borne < pTmp[dim]) {
                borne = pTmp[dim];
                idRes = t;
            }
        }
        else
        {
            if (borne > pTmp[dim]) {
                borne = pTmp[dim];
                idRes = t;
            }
        }
    }

    return idRes;
}

/*------------------------------------------------------------------------------
 * Charge un fichier .txt ou .xyz à partir de @filename
 * (attention ne gère pas encore les couleurs).
 * ----------------------------------------------------------------------------*/
vector<QVector3D> Contour::loadCloud(const string &filename)
{
    qDebug() << "\t<" << __FUNCTION__ << ">";

    ifstream monFlux(filename);  //Ouverture d'un fichier en lecture
    if(!monFlux) {
        qWarning()<< "ERREUR: Impossible d'ouvrir le fichier en lecture." << endl;
    }

    vector<QVector3D> myVec;

    QVector3D P;
    int dim=0;
    double tmp;
    while (monFlux >> tmp)
    {
        if (dim>=3)
        {
            myVec.push_back(P);
            dim=0;
        }
        switch(dim)
        {
        case 0:
            P.setX(tmp);
            //            cout << "x = " << tmp << "\t";
            break;
        case 1:
            P.setY(tmp);
            //            cout << "y = " << tmp << "\t";
            break;
        case 2:
            P.setZ(tmp);
            //            cout << "z = " << tmp << endl;
        default:
            break;
        }
        dim++;
    }


    qDebug() << "\t</" << __FUNCTION__ << ">";

    return myVec;
}

Contour::Contour(MyMesh &_mesh, char *path) :
    mesh(_mesh)
{
    qDebug() << "<" << __FUNCTION__ << ">";

    vector<QVector3D> myVec = loadCloud(path);

    vector<int> tmp;

    for (auto v : myVec)
    {
        MyMesh::Point P (v.x(), v.y(), v.z());
        int numVertex = UtilsMesh::find_near_vertex_of_point(&mesh, P);
        if ( ! Utils::is_in_vector(this->verticesContour, static_cast<unsigned>(numVertex))) {
            //this->add_vertex(numVertex);
            tmp.push_back(numVertex);
        }
    }
    Utils::suppr_occur(tmp);

    /////////////// V2 ////////////////
    int id = 0;
    for (unsigned i=0; i<tmp.size()  ; i++)
    {
        //        qDebug() << "\t\ti=" << i;
        if (i==0) {
            this->add_vertex(tmp[i]);
            id = tmp[i];
        }
        else
        {
            id = search_min_dist_vertex_from_vertex(tmp, id);
            if (id<0) {
                qWarning() << "in" << __FUNCTION__ << ": id < 0";
                exit (2);
            }
            add_vertex(id);
        }
    }

    // V3
    //    for (auto t : tmp) {
    //        this->add_vertex(t);
    //    }

    qDebug() << "</" << __FUNCTION__ << ">";
}

unsigned Contour::get_start()   {   return startPoint;  }
unsigned Contour::get_end()     {   return endPoint;    }

vector<unsigned> Contour::get_contour() {   return verticesContour; }
void Contour::set_contour(vector<unsigned> tmp)
{
    verticesContour.clear();
    for (auto v : tmp) {
        add_vertex(v);
    }
}


///////////////////////////// AUTRES   ////////////////////////////////////////

/*------------------------------------------------------------------------------
 * Réduit le nombre de sommets dans @verticesContour
 * à partir d'une valeur modulo.
 * (exemple: on ne garde que les id de sommets (dans le tableau@verticesContour)
 * dont le modulo @moduloVal vaut 0)
 * ----------------------------------------------------------------------------*/
void Contour::reduct(int moduloVal)
{
    if (moduloVal < 0)     moduloVal *= -1;
    if (moduloVal == 0)    moduloVal = 1;

    vector<unsigned> tmp;
    for (unsigned v=0; v<verticesContour.size(); v++)
    {
        if (v%moduloVal==0) {
            tmp.push_back(verticesContour[v]);
        }
    }
    this->set_contour(tmp);
}

void Contour::display(int profDisplay, bool flagColor)
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
    int cpt=0;
    for (auto p : verticesContour)
    {
        qDebug() << cprof << "\tsommet =" << p;
        if (flagColor) {
            VertexHandle vh = mesh.vertex_handle(p);
            mesh.data(vh).thickness = 10;
            mesh.set_color(vh, MyMesh::Color(cpt, 255 - (4*cpt), 0));
            cpt++;
        }
    }
    qDebug() << cprof << "\tend =" << endPoint;

    qDebug() << cprof << "</" << __FUNCTION__ << ">";
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

    //    this->display(1);

    unsigned curVertex = startPoint;
    unsigned cpt=0;
    int curVertex2 = verticesContour[cpt+1];

    qDebug() << "\t\tchargement draw: INIT ->" << verticesContour.size() << "sommets à calculer";
    LiveWire lW(*_mesh, curVertex, _sightPoint);

    if (curVertex2==endPoint && curVertex2!=startPoint)
    {
        //        qDebug() << "\t\tpremier if";
        qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
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
            //            qDebug() << "\t\tboucle cpt=" << cpt;
            qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
            curVertex2 = verticesContour[cpt+1];
            lW.update_vertexSeed(curVertex, curVertex2);
            lW.draw(curVertex2);
            curVertex = curVertex2;
            cpt++;
        }
    }


    qDebug() << "\t\tchargement draw:" << cpt<<"/"<<verticesContour.size();
    lW.update_vertexSeed(endPoint, startPoint, true);
    lW.draw(startPoint);
    qDebug() << "\t\tchargement draw:" << verticesContour.size()<<"/"<<verticesContour.size();

    qDebug() << "\t</" << __FUNCTION__ << ">";
}
