#include "livewire.h"

#include <QDebug>


///////////////////////////////  CONSTRUCTEURS   ////////////////////////////////////////////////


void LiveWire::init_criterions()
{
    criteres = vector<int>(nbMaxCrit, -1);
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
    //    unsigned mySize = mesh.n_edges();
    vector<double> tmpVec;
    for (unsigned i=0; i<nb_criterions_preload; i++) {
        tabCosts.push_back(tmpVec);
    }
}

/*------------------------------------------------------------------------------
 * Faire très attention à bien définir la variable @nb_criterions_preload
 * ----------------------------------------------------------------------------*/
LiveWire::LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint) :
    mesh(_mesh), vertexSeed(_vertexSeed), sightPoint(_sightPoint)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    init_criterions();

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

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

vector<int> LiveWire::get_paths()  {   return paths;   }

void LiveWire::update_vertexSeed(int _vertexSeed)
{
    vertexSeed = _vertexSeed;
    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
        myDijkstra.dijkstra(&mesh, vertexSeed);
        //        vector<int> dijkstraPaths = myDijkstra.get_paths();
    }
    build_paths(vertexSeed);
}


//////////////////////////////////  CRITERES PRELOAD  //////////////////////////////////

double LiveWire::criterion_length(EdgeHandle eh) {
    return mesh.calc_edge_length(eh);
}

double LiveWire::criterion_diedral_angle(EdgeHandle eh) {
    return mesh.calc_dihedral_angle(eh);
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
    vector<int> path = myDijkstra.get_currentPath();

    int numEdge = eh.idx();
    double bestCost = static_cast<double>(INT_MAX);
    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);
    for (auto p : path)
    {
        if (p==numEdge) {
            return 0.0;
        }
        EdgeHandle ehTest = mesh.edge_handle(p);
        MyMesh::Point pTest = mesh.calc_edge_midpoint(ehTest);
        double distEuclid = Utils::distance_euclidienne(myP[0], pTest[0],
                                                        myP[1], pTest[1]);
        if (bestCost <= distEuclid) {
            bestCost = distEuclid;
        }
    }

    return bestCost;
}


///////////////////////////////////  ALGO   ////////////////////////////////////////////////

//double LiveWire::cost_function(int numEdgeCur, int numEdgeNeigh)
//{
//    double cost = 0.0;
//    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
//    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

//    // Length
//    cost = mesh.calc_edge_length(ehNeigh);
//    // dihedral angle
//    //    cost *= mesh.calc_dihedral_angle(ehNeigh);
//    // Normal orientation
//    //    cost*=normal_orientation(numEdgeNeigh, sightPoint);

//    // TESTS
//    MyMesh::Point myNorm;
//    myNorm = mesh.calc_edge_vector(ehNeigh);
//    //    mesh.has_halfedge_normals ();
//    // const Normal & 	normal (HalfedgeHandle _heh) const

//    return cost;
//}

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

    /////////////////// COUT CRITERES PRECHARGES ///////////////////////////////////
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

void LiveWire::build_paths(int _vertexSeed)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    // Init
    EdgeHandle ehTmp = UtilsMesh::get_next_eh_of_vh(&mesh, vertexSeed);
    edgeSeed = ehTmp.idx();

    qDebug() << "\t\t\tvertexSeed =" << vertexSeed;
    qDebug() << "\t\t\tedgeSeed =" << edgeSeed;

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

        //        qDebug() << "\t\tcurEdge =" << curEdge;
        //        qDebug() << activeList;
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
//    // TMP anti bug
//    if (edgeBegin!=mesh.n_edges()-1)
//        paths[edgeBegin] = edgeBegin+1;
    qDebug() << "\t\t</" << __FUNCTION__ << ">";
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
    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
        myDijkstra.calc_path(&mesh, vertex2);
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













