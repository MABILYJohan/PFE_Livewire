#include "livewire.h"

#include <QDebug>
#include <cstring>

//////////////////////////  CONSTRUCTEURS / REGLAGES   ///////////////////////////////////////

LiveWire::LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint) :
    mesh(_mesh), vertexSeed(_vertexSeed), sightPoint(_sightPoint)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    qsrand(time(NULL));
    init_criterions();
    display_criterions(3);

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

vector<int> LiveWire::get_paths()  {   return paths;   }

/*------------------------------------------------------------------------------
 * A défaut de le faire dans l'interface, c'est ici qu'on choisit
 * les critères souhaités, et qu'on initie les tableaux de préchargement.
 * ----------------------------------------------------------------------------*/
void LiveWire::init_criterions()
{
    criteres.clear();
    criteres.push_back(LENGTH);
    criteres.push_back(DIEDRAL);
    criteres.push_back(NORMAL_OR);
    criteres.push_back(VISIBILITY);
    //    criteres.push_back(CURVATURE);

    unsigned nb_criterions_preload=0;
    for(auto c : criteres) {
        if (c==LENGTH ||  c==DIEDRAL    || c==NORMAL_OR) {
            nb_criterions_preload++;
        }
    }

    tabCosts.clear();
    vector<double> tmpVec;
    for (unsigned i=0; i<nb_criterions_preload; i++) {
        tabCosts.push_back(tmpVec);
    }

    unsigned cpt=0;

    qDebug() << "\t\t\tchargement LW:" << 0 <<"/"<<mesh.n_edges();
    for (MyMesh::EdgeIter curEdge = mesh.edges_begin(); curEdge != mesh.edges_end(); curEdge++)
    {
        cpt=0;
        EdgeHandle eh = *curEdge;
        if (Utils::is_in_vector(criteres, static_cast<int>(LENGTH))) {
            tabCosts[cpt].push_back(criterion_length(eh)); cpt++;
        }
        if (Utils::is_in_vector(criteres, static_cast<int>(DIEDRAL))) {
            tabCosts[cpt].push_back(criterion_diedral_angle(eh)); cpt++;
        }
        if (Utils::is_in_vector(criteres, static_cast<int>(NORMAL_OR))) {
            tabCosts[cpt].push_back(criterion_normal_orientation(eh, sightPoint)); cpt++;
        }
        if (eh.idx()%1000 == 0)
            qDebug() << "\t\t\tchargement LW:" << eh.idx()+1<<"/"<<mesh.n_edges();
    }
    qDebug() << "\t\t\tchargement LW:" << mesh.n_edges()<<"/"<<mesh.n_edges();
}

/*------------------------------------------------------------------------------
 * Met à jour le sommet de départ @_vertexSeed et met à jour Dijkstra
 * si critère de visibilité puis refait les chemins avec critères.
 * @vertexNext pour le critère de visibilité avec dijkstra.
 * ----------------------------------------------------------------------------*/
void LiveWire::update_vertexSeed(int _vertexSeed, int vertexNext)
{
    vertexSeed = _vertexSeed;
    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
        myDijkstra.dijkstra(&mesh, vertexSeed);
        //        vector<int> dijkstraPaths = myDijkstra.get_paths();
    }
    build_paths(vertexNext);
}

void LiveWire::display_criterions(int profDisplay)
{
    if (profDisplay<0)  profDisplay=0;
    if (profDisplay>5)  profDisplay=5;
    char prof[profDisplay];
    for (int i=0; i< profDisplay; i++) {
        prof[i] = '\t';
    }
    char *cprof = prof;

    qDebug() << cprof << "<" << __FUNCTION__ << ">";

    for(auto c : criteres) {
        switch (c) {
        case(LENGTH):
            qDebug() << cprof <<"\tLENGTH";
            break;
        case(DIEDRAL):
            qDebug() << cprof <<"\tDIEDRAL";
            break;
        case(NORMAL_OR):
            qDebug() << cprof <<"\tNORMAL_ORIENTATION";
            break;
        case(VISIBILITY):
            qDebug() << cprof <<"\tVISIBILITY";
            break;
        case(CURVATURE):
            qDebug() << cprof <<"\tCURVATURE";
            break;
        default:
            break;
        }
    }

    qDebug() << cprof << "</" << __FUNCTION__ << ">";
}


//////////////////////////////////  CRITERES PRELOAD  //////////////////////////////////

double LiveWire::criterion_length(EdgeHandle eh) {
    return mesh.calc_edge_length(eh);
}

double LiveWire::criterion_diedral_angle(EdgeHandle eh) {
    // Voir si besoin que le coût inférieur soit l'angle le plus grand
    return fabs(mesh.calc_dihedral_angle(eh));
}

double LiveWire::criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint)
{
    double cost=0.0;

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


//////////////////////////////////  CRITERES AUTRES //////////////////////////////////

double LiveWire::criterion_visibility(EdgeHandle eh)
{
    vector<int> dijkstraPaths = myDijkstra.get_paths();
    if (dijkstraPaths.empty()) {
        qWarning() << "Warning in " << __FUNCTION__
                   << "dijkstraPaths is empty";
        exit(1);
    }

    vector<int> pathEdges = myDijkstra.get_currentPath();
    int numEdge = eh.idx();
    double bestCost = static_cast<double>(INT_MAX);
    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);

    for (auto id : pathEdges)
    {
        if (id==numEdge) {
            return 0.0;
        }
        EdgeHandle ehTest = mesh.edge_handle(id);
        MyMesh::Point pTest = mesh.calc_edge_midpoint(ehTest);
        double distEuclid = Utils::distance_euclidienne(myP[0], pTest[0],
                myP[1], pTest[1],
                myP[2], pTest[2]);
        if (bestCost >= distEuclid) {
            bestCost = distEuclid;
        }
    }

    return bestCost;
}


///////////////////////////////////  ALGO   ////////////////////////////////////////////////

double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
{
    //    qDebug() << "\t\t\t<" << __FUNCTION__ << ">";

    double cost = 1.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    /////////////////// COUT CRITERES PRECHARGES ///////////////////////////////////
    if (!tabCosts.empty()) {
        for (auto listCout : tabCosts){
            cost *= listCout[numEdgeNeigh];
        }
    }

    /////////////////// COUTS AUTRES ///////////////////////////////////
    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
        cost *= criterion_visibility(ehNeigh);
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

/*------------------------------------------------------------------------------
 * Construit les chemins pour le livewire (dans @paths).
 * @vertexNext pour le critère de visibilité avec dijkstra.
 * Pensez au cas où à bien mettre à jour @vertexSeed avant d'utiliser la focntion.
 * ----------------------------------------------------------------------------*/
void LiveWire::build_paths(int vertexNext)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    // Init
    EdgeHandle ehTmp = UtilsMesh::get_next_eh_of_vh(&mesh, vertexSeed);
    edgeSeed = ehTmp.idx();

    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    costEdges[edgeSeed] = 0.0;
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;   activeList.push_back(edgeSeed);
    paths = vector<int>(mesh.n_edges(), -1);

    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
        myDijkstra.calc_path(&mesh, vertexNext);
    }

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    while (!activeList.empty())
    {
        int curEdge = get_minCostEdge_from_activeList(activeList, costEdges);
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
                activeList.push_back(edgeNeigh);
            }
        }
    }

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}


///////////////////////////////////  AUTRES   ////////////////////////////////////////////////

/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned vertex2)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    //    if (tabCosts.empty()) {
    //        qWarning() << "in" << __FUNCTION__ << ": tabCosts is empty";
    //        return;
    //    }

    int red = Utils::randInt(0, 255);
    int blue = Utils::randInt(0, 255);
    int green = Utils::randInt(0, 255);

    EdgeHandle eh1 = mesh.edge_handle(edgeSeed);
    EdgeHandle eh2 = UtilsMesh::get_next_eh_of_vh(&mesh, vertex2);
    unsigned curEdge = eh2.idx();
    vector<EdgeHandle> ehs;

    while (static_cast<int>(curEdge) != edgeSeed)
    {
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

    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}













