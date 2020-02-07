#include "livewire.h"

#include <QDebug>


///////////////////////////////  CONSTRUCTEURS   ////////////////////////////////////////////////

void LiveWire::init_criterions(unsigned nbCriterions)
{
    tabCosts.clear();
    //    unsigned mySize = mesh.n_edges();
    vector<double> tmpVec;
    for (unsigned i=0; i<nbCriterions; i++) {
        tabCosts.push_back(tmpVec);
    }
}

/*------------------------------------------------------------------------------
 * Faire très attention à bien définir la variable @nb_criterions
 * ----------------------------------------------------------------------------*/
LiveWire::LiveWire(MyMesh &_mesh, MyMesh::Point _sightPoint) :
    mesh(_mesh), sightPoint(_sightPoint)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    unsigned nb_criterions = 2;
    if (nb_criterions <= 0) {
        qWarning() << "in" << __FUNCTION__ << ": nb_criterions can't be inferior to 1:"
                   << "\nnb_criterions =" << nb_criterions << "not accepted";
        return;
    }
    init_criterions(nb_criterions);
    unsigned cpt=0;


    qDebug() << "\t\t\tchargement LW:" << 0 <<"/"<<mesh.n_edges();
    for (MyMesh::EdgeIter curEdge = mesh.edges_begin(); curEdge != mesh.edges_end(); curEdge++)
    {
        cpt=0;
        EdgeHandle eh = *curEdge;

        tabCosts[cpt].push_back(criterion_length(eh)); cpt++;
        tabCosts[cpt].push_back(criterion_diedral_angle(eh)); cpt++;
        if (eh.idx()%1000 == 0)
            qDebug() << "\t\t\tchargement LW:" << eh.idx()+1<<"/"<<mesh.n_edges();
    }
    qDebug() << "\t\t\tchargement LW:" << mesh.n_edges()<<"/"<<mesh.n_edges();

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

vector<int> LiveWire::get_paths()  {   return paths;   }


//////////////////////////////////  CRITERES   ///////////////////////////////////////////////////

double LiveWire::criterion_length(EdgeHandle eh) {
    return mesh.calc_edge_length(eh);
}

double LiveWire::criterion_diedral_angle(EdgeHandle eh) {
    return mesh.calc_dihedral_angle(eh);
}

double LiveWire::normal_orientation(int numEdge, MyMesh::Point _sightPoint)
{
    double cost=0.0;

    EdgeHandle eh = mesh.edge_handle(numEdge);

    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);
    MyMesh::Point myVec = myP - _sightPoint;

    MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 0);
    FaceHandle fh1 = mesh.face_handle(heh1);
    MyMesh::Normal n1 = mesh.calc_face_normal(fh1);
    float angle1 = acos(dot(n1, myVec));
    cost+=angle1;

    if ( ! mesh.is_boundary(eh) )
    {
        MyMesh::HalfedgeHandle heh2 = mesh.halfedge_handle(eh, 1);
        FaceHandle fh2 = mesh.face_handle(heh2);
        MyMesh::Normal n2 = mesh.calc_face_normal(fh2);
        float angle2 = acos(dot(n2, myVec));
        cost+=angle2;
    }

    return cost;
}


///////////////////////////////////  ALGO   ////////////////////////////////////////////////

double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    //    qDebug() << "\t\t\t<" << __FUNCTION__ << ">";

    if (tabCosts.empty()) {
        qWarning() << "in" << __FUNCTION__ << ": tabCosts is empty";
        exit (1);
    }
    double cost = 0.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    // Init pour éviter de multiplier par 0
    cost = tabCosts[0][numEdgeNeigh];
    bool flagFirst=true;
    for (auto listCout : tabCosts)
    {
        if (flagFirst) {
            flagFirst = false;
            continue;
        }
        cost *= listCout[numEdgeNeigh];
    }

    //    qDebug() << "\t\t\t</" << __FUNCTION__ << ">";
    return cost;
}

unsigned get_minCostEdge_from_activeList(vector<int> activeList, vector<double> costEdges)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    unsigned numEdge=0;
    double min = static_cast<double>(INT_MAX);
    for (auto e : activeList) {
        if (costEdges[e] <= min) {
            min = costEdges[e];
            numEdge = e;
        }
    }
    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
    return numEdge;
}

void LiveWire::build_paths(int _vertexSeed)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    vertexSeed = _vertexSeed;

    // Init
    EdgeHandle ehTmp = UtilsMesh::get_next_eh_of_vh(&mesh, vertexSeed);
    edgeSeed = ehTmp.idx();

    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeSeed] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;   activeList.push_back(edgeSeed);
    paths = vector<int>(mesh.n_edges(), -1);

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    while (!activeList.empty())
    {
        int curEdge = get_minCostEdge_from_activeList(activeList, costEdges);
        //        int curEdge = Utils::get_min(costEdges);
        Utils::erase_elt(activeList, curEdge);
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
    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}


///////////////////////////////////  AUTRES   ////////////////////////////////////////////////

/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned vertex2)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    if (tabCosts.empty()) {
        qWarning() << "in" << __FUNCTION__ << ": tabCosts is empty";
        return;
    }

    srand(0);
    int red = Utils::randInt(0, 255);
    int blue = Utils::randInt(0, 255);
    int green = Utils::randInt(0, 255);

    EdgeHandle eh1 = mesh.edge_handle(edgeSeed);
    //    EdgeHandle eh2 = mesh.edge_handle(vertex2);
    EdgeHandle eh2 = UtilsMesh::get_next_eh_of_vh(&mesh, vertex2);

    //    qDebug() << "\t\tTest";

    unsigned curEdge = vertex2;
    //    qDebug() << "\t\t\tedgeSeed=" << edgeSeed;
    //    qDebug() << "\t\t\tedge2=" << edge2;

    vector<EdgeHandle> ehs;

    //    qDebug() << "\t\t\tcurEdge=" << curEdge;
    while (static_cast<int>(curEdge) != edgeSeed)
    {
        //        qDebug() << "\t\tcurEdge =" << curEdge;

        ehs.clear();
        ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            if (eh.idx()==paths[curEdge]) {
                mesh.set_color(eh, MyMesh::Color(red, blue, green));
                mesh.data(eh).thickness = 6;
            }
        }
        if (ehs.empty())    break;
        curEdge = paths[curEdge];
        //        qDebug() << "\t\t\tcurEdge=" << curEdge;
    }

    // point de départ et point d'arrivée en rouge et en gros
    mesh.set_color(eh1, MyMesh::Color(255, 0, 0));
    mesh.set_color(eh2, MyMesh::Color(255, 0, 0));
    mesh.data(eh1).thickness = 8;
    mesh.data(eh2).thickness = 8;
    VertexHandle vh1 = mesh.vertex_handle(vertexSeed);
    VertexHandle vh2 = mesh.vertex_handle(vertex2);
    mesh.set_color(vh1, MyMesh::Color(255, 0, 0));
    mesh.set_color(vh2, MyMesh::Color(255, 0, 0));
    mesh.data(vh1).thickness = 20;
    mesh.data(vh2).thickness = 20;

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}













