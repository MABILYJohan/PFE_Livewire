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
    // ATTENTION à l'ordre
    // Mettre en commentaires les critères non souhaités
    criteres.clear();
    criteres.push_back(LENGTH);
    criteres.push_back(DIEDRAL);
    criteres.push_back(CURVATURE);
    criteres.push_back(NORMAL_OR);
    criteres.push_back(VISIBILITY);
    criteres.push_back(STROKE_DIST);

    unsigned nb_criterions_preload=0;
    for(auto c : criteres) {
        if (c==LENGTH ||  c==DIEDRAL    || c==NORMAL_OR || c==CURVATURE) {
            nb_criterions_preload++;
        }
    }

    // INIT MIN ET MAX CURVATURE pour plage [0,1]
    vector<double> tabcurv;
    for (MyMesh::VertexIter curVert = mesh.vertices_begin(); curVert != mesh.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        tabcurv.push_back(abs(K_Curv(&mesh, vh.idx())));
    }
    this->minCurv = *std::min_element(tabcurv.begin(), tabcurv.end());
    this->maxCurv = *std::max_element(tabcurv.begin(), tabcurv.end());

    tabCosts.clear();
    vector<double> tmpVec;
    for (unsigned i=0; i<nb_criterions_preload; i++) {
        tabCosts.push_back(tmpVec);
    }

    unsigned cpt=0;

    qDebug() << "\t\t\tchargement LW POIDS DE CRITERES A PRECHARGER";
    qDebug() << "\t\t\tchargement LW:" << 0 <<"/"<<mesh.n_edges();
    for (MyMesh::EdgeIter curEdge = mesh.edges_begin(); curEdge != mesh.edges_end(); curEdge++)
    {
        // DOIVENT ETRE AJOUTES DANS LE MEME ORDRE QUE LE TABLEAU @criteres
        cpt=0;
        EdgeHandle eh = *curEdge;
        if (Utils::is_in_vector(criteres, static_cast<int>(LENGTH))) {
            tabCosts[cpt].push_back(criterion_length(eh)); cpt++;
        }
        if (Utils::is_in_vector(criteres, static_cast<int>(DIEDRAL))) {
            tabCosts[cpt].push_back(criterion_diedral_angle(eh)); cpt++;
        }
        if (Utils::is_in_vector(criteres, static_cast<int>(CURVATURE))) {
            tabCosts[cpt].push_back(criterion_curvature(eh)); cpt++;
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
void LiveWire::update_vertexSeed(int _vertexSeed, int vertexNext, bool close)
{
    vertexSeed = _vertexSeed;
    if (! close)
    {
        if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))
                || Utils::is_in_vector(criteres, static_cast<int>(STROKE_DIST))) {
            myDijkstra.dijkstra(&mesh, vertexSeed);
            //        vector<int> dijkstraPaths = myDijkstra.get_paths();
        }
    }
    build_paths(vertexNext, close);
}

void LiveWire::display_criterions(int profDisplay)
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
        case(STROKE_DIST):
            qDebug() << cprof <<"\tSTROKE_DIST";
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
    return fabs(2*M_PI - mesh.calc_dihedral_angle(eh));
    //    return fabs(mesh.calc_dihedral_angle(eh));
}

double LiveWire::criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint)
{
    double cost=0.0;

    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);
    MyMesh::Point myVec = myP - _sightPoint;

    MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 0);
    FaceHandle fh1 = mesh.face_handle(heh1);
    MyMesh::Normal n1 = mesh.calc_face_normal(fh1);
    //    float angle1 = acos(dot(n1, myVec));
    QVector3D test1(n1[0], n1[1], n1[2]);
    QVector3D test2(myVec[0], myVec[1], myVec[2]);
    float angle1 = QVector3D::dotProduct(test1, test2);
    cost+=angle1;

    if ( ! mesh.is_boundary(eh) )
    {
        MyMesh::HalfedgeHandle heh2 = mesh.halfedge_handle(eh, 1);
        FaceHandle fh2 = mesh.face_handle(heh2);
        MyMesh::Normal n2 = mesh.calc_face_normal(fh2);
        test1 = QVector3D(n2[0], n2[1], n2[2]);
        test2 = QVector3D(myVec[0], myVec[1], myVec[2]);
        float angle2 = QVector3D::dotProduct(test1, test2);
        //        float angle2 = acos(dot(n2, myVec));
        cost+=angle2;
    }

    return fabs(2*M_PI - cost);
    //    return fabs(cost);
}

float LiveWire::angleEE(MyMesh* _mesh, int vertexID, int faceID)
{
    FaceHandle face_h = _mesh->face_handle(faceID);
    QVector<QVector3D> points;
    QVector3D point_origine;

    // on cherche le point d'origine
    for(MyMesh::FaceVertexIter curVer = _mesh->fv_begin(face_h); curVer.is_valid(); curVer++) {
        VertexHandle vertex_h = *curVer;

        if(vertex_h.idx() == vertexID) {
            point_origine = QVector3D(_mesh->point(vertex_h)[0],_mesh->point(vertex_h)[1],_mesh->point(vertex_h)[2]);
        }
        else {
            points.push_back(QVector3D(_mesh->point(vertex_h)[0],_mesh->point(vertex_h)[1],_mesh->point(vertex_h)[2]));
        }
    }

    QVector3D vecteur1 = points[1] - point_origine;
    QVector3D vecteur2 = points[0] - point_origine;
    vecteur1.normalize();
    vecteur2.normalize();

    return acos(QVector3D::dotProduct(vecteur1, vecteur2));
}

float LiveWire::faceArea(MyMesh* _mesh, int faceID)
{

    FaceHandle face_h = FaceHandle(faceID);
    QVector<MyMesh::Point> points;
    for(MyMesh::FaceVertexIter curVer = _mesh->fv_iter(face_h); curVer.is_valid(); curVer++) {
        VertexHandle vertex_h = *curVer;
        points.push_back(_mesh->point(vertex_h));
    }

    float aire = norm((points[1] - points[0]) % (points[2] - points[0])) / 2.f;
    return aire;
}

float LiveWire::aire_barycentrique(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float area = 0.f;
    for(MyMesh::VertexFaceIter vfit = _mesh->vf_iter(vh); vfit.is_valid(); vfit++){
        area += faceArea(_mesh,(*vfit).idx());
    }
    return area / 3.f;
}

double LiveWire::K_Curv(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float a = 1.f / aire_barycentrique(_mesh, vh.idx());
    float theta = 0.f;
    for (MyMesh::VertexFaceCWIter vf_it = _mesh->vf_cwiter(vh); vf_it.is_valid(); vf_it++)
        {
            FaceHandle fh = *vf_it;
            theta += angleEE(_mesh, vh.idx(), fh.idx());
        }
    float b = 2.f*M_PI - theta;
    float K = a*b;
    return K;
}

double LiveWire::criterion_curvature(EdgeHandle eh)
{
    VertexHandle vh_commun, vh_suivant;
    UtilsMesh::get_vh_of_edge(&mesh, eh.idx(), vh_commun, vh_suivant);
    double K_commun = abs(K_Curv(&mesh, vh_commun.idx()));
    double K_suivant = abs(K_Curv(&mesh, vh_suivant.idx()));

    //plage [0,1]
    K_commun = (K_commun-minCurv)/(maxCurv-minCurv);
    K_suivant = (K_suivant-minCurv)/(maxCurv-minCurv);
    double cost = (K_commun+K_suivant)/2.0;
    cost = 1-cost;
    return cost;
}


//////////////////////////////////  CRITERES AUTRES //////////////////////////////////

double LiveWire::criterion_souplesse_edge(EdgeHandle eh1, EdgeHandle eh2)
{
    double cost=0.0;

    MyMesh::Point vec1 = mesh.calc_edge_vector(eh1);
    MyMesh::Point vec2 = mesh.calc_edge_vector(eh2);

    vec1.normalize();
    vec2.normalize();

    QVector3D v1 = UtilsMesh::to_qvector3D(vec1);
    QVector3D v2 = UtilsMesh::to_qvector3D(vec2);
    float angle = QVector3D::dotProduct(v1, v2);
    if (fabs(angle) <= 45.0) {
        cost = fabs(2*M_PI - angle);
        if (cost==0.0)  cost = 0.01;
    }
    else cost = 1.0;

    return cost;
}

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
    double distMin = static_cast<double>(INT_MAX);
    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);

    double distMax = 500;
    for (auto idEdgePath : pathEdges)
    {
        if (idEdgePath==numEdge) {
            //            return static_cast<double>(INT_MAX)-1.0;
            return distMax;
        }

        EdgeHandle ehPath = mesh.edge_handle(idEdgePath);
        MyMesh::Point pTest = mesh.calc_edge_midpoint(ehPath);
        double distEuclid = Utils::distance_euclidienne(myP[0], myP[1], myP[2],
                pTest[0], pTest[1], pTest[2]);

        if (distMin >= distEuclid) {
            distMin = distEuclid;
        }

        HalfedgeHandle heh = mesh.halfedge_handle(ehPath, 0);
        MyMesh::Point from, to;
        to = mesh.point(mesh.to_vertex_handle(heh));
        from = mesh.point(mesh.from_vertex_handle(heh));

        HalfedgeHandle start = mesh.halfedge_handle(EdgeHandle(pathEdges.front()), 0);
        MyMesh::Point start_from, start_to;
        start_to = mesh.point(mesh.to_vertex_handle(start));
        start_from = mesh.point(mesh.from_vertex_handle(start));

        HalfedgeHandle end = mesh.halfedge_handle(EdgeHandle(pathEdges.back()), 0);
        MyMesh::Point end_from, end_to;
        end_to = mesh.point(mesh.to_vertex_handle(end));
        end_from = mesh.point(mesh.from_vertex_handle(end));

        QVector3D a(to.data()[0], to.data()[1], to.data()[2]);
        QVector3D b(start_to.data()[0], start_to.data()[2], start_to.data()[3]);

        double max_dist = Utils::distance_euclidienne(to.data()[0], to.data()[1], to.data()[2], start_to.data()[0], start_to.data()[2], start_to.data()[3]);

        max_dist = max(max_dist, Utils::distance_euclidienne(to.data()[0], to.data()[1], to.data()[2], start_from.data()[0], start_from.data()[2], start_from.data()[3]));
        max_dist = max(max_dist, Utils::distance_euclidienne(to.data()[0], to.data()[1], to.data()[2], end_from.data()[0], end_from.data()[2], end_from.data()[3]));
        max_dist = max(max_dist, Utils::distance_euclidienne(to.data()[0], to.data()[1], to.data()[2], end_to.data()[0], end_to.data()[2], end_to.data()[3]));

        max_dist = max(max_dist, Utils::distance_euclidienne(from.data()[0], from.data()[1], from.data()[2], end_to.data()[0], end_to.data()[2], end_to.data()[3]));
        max_dist = max(max_dist, Utils::distance_euclidienne(from.data()[0], from.data()[1], from.data()[2], start_to.data()[0], start_to.data()[2], start_to.data()[3]));
        max_dist = max(max_dist, Utils::distance_euclidienne(from.data()[0], from.data()[1], from.data()[2], start_from.data()[0], start_from.data()[2], start_from.data()[3]));
        max_dist = max(max_dist, Utils::distance_euclidienne(from.data()[0], from.data()[1], from.data()[2], end_from.data()[0], end_from.data()[2], end_from.data()[3]));

        distMin = max(max_dist, distMin);
    }

    double cost = 0;
    if(distMin >= rad_thickness)
    {
       cost = rad_thickness/(distMin*distMin);
    }

    else
    {
        cost = rad_thickness/(distMin);
    }


    return cost;
}

double LiveWire::criterion_stroke_distance(EdgeHandle eh)
{
    vector<int> dijkstraPaths = myDijkstra.get_paths();
    if (dijkstraPaths.empty()) {
        qWarning() << "Warning in " << __FUNCTION__
                   << "dijkstraPaths is empty";
        exit(1);
    }

    vector<int> pathEdges = myDijkstra.get_currentPath();
    int numEdge = eh.idx();
    //    double distMin = static_cast<double>(INT_MAX);
    MyMesh::Point myP = mesh.calc_edge_midpoint(eh);

    double distMax = 100.0;
    for (auto idEdgePath : pathEdges)
    {
        if (idEdgePath==numEdge) {
            //            return static_cast<double>(INT_MAX)-1.0;
            break;
        }

        EdgeHandle ehPath = mesh.edge_handle(idEdgePath);
        MyMesh::Point pTest = mesh.calc_edge_midpoint(ehPath);
        double distEuclid = Utils::distance_euclidienne(myP[0], myP[1], myP[2],
                pTest[0], pTest[1], pTest[2]);

        //Looking for approximative maximal distance from the edge to the center-line of the stroke
        if (distMax < distEuclid) {
            distMax = distEuclid;
        }
    }

    double cost = (rad_thickness + distMax)/rad_thickness;

    return cost;

}
///////////////////////////////////  ALGO   ////////////////////////////////////////////////

double LiveWire::cost_function(int numVertex, int numEdgeNeigh, bool close)
{
    //    qDebug() << "\t\t\t<" << __FUNCTION__ << ">";

    double cost = 1.0;
    // EdgeHandle ehCur = mesh.edge_handle(numEdgeCur);
    EdgeHandle ehNeigh = mesh.edge_handle(numEdgeNeigh);

    /////////////////// COUT CRITERES PRECHARGES ///////////////////////////////////
    int cpt=LENGTH;
    if (!tabCosts.empty()) {
        for (auto listCout : tabCosts){
            if (close
                    &&  Utils::is_in_vector(criteres, static_cast<int>(NORMAL_OR))
                    &&  cpt==NORMAL_OR) {
                continue;
            }
            cost *= listCout[numEdgeNeigh];
        }
        cpt++;
    }

    /////////////////// COUTS AUTRES ///////////////////////////////////
    if (close)
    {
        if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))) {
            cost *= criterion_visibility(ehNeigh);
        }

        if (Utils::is_in_vector(criteres, static_cast<int>(STROKE_DIST))) {
            cost *= criterion_stroke_distance(ehNeigh);
        }
    }

    //    // TMP
    //    if (numVertex != vertexSeed)
    //    {
    //        int numPreviousVertex = paths[numVertex];
    //        VertexHandle vhPrev = mesh.vertex_handle(numPreviousVertex);
    //        int numPreviousEdge=-1;
    //        for (MyMesh::VertexEdgeCWIter ve_it = mesh.ve_cwiter(vhPrev); ve_it.is_valid(); ve_it++)
    //        {
    //            EdgeHandle ehTmp = *ve_it;
    //            VertexHandle vh1, vh2;
    //            UtilsMesh::get_vh_of_edge(&mesh, ehTmp.idx(), vh1, vh2);
    //            if (vh1.idx() == numPreviousVertex
    //                    || vh2.idx() == numPreviousVertex) {
    //                numPreviousEdge = ehTmp.idx();
    //                break;
    //            }
    //        }
    //        EdgeHandle ehPrev = mesh.edge_handle(numPreviousEdge);
    //        cost *= criterion_souplesse_edge(ehNeigh, ehPrev);
    //    }


    //    qDebug() << "\t\t\t</" << __FUNCTION__ << ">";
    return cost;
}

unsigned get_minCostToVertex_from_activeList(vector<int> activeList, vector<double> costsToVertices)
{
    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    unsigned numVertex=0;
    double min = static_cast<double>(INT_MAX);
    for (auto v : activeList) {
        if (costsToVertices[v] <= min) {
            min = costsToVertices[v];
            numVertex = v;
        }
    }
    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
    return numVertex;
}

/*------------------------------------------------------------------------------
 * Retourne l'arête autour de @vertexStart, la plus proche du sommet
 * d'indice @vertexNext
 * ----------------------------------------------------------------------------*/
EdgeHandle LiveWire::get_edge_seed(int vertexStart, int vertexEnd)
{
    VertexHandle vhStart = mesh.vertex_handle(vertexStart);
    VertexHandle vhTmp = mesh.vertex_handle(vertexEnd);
    MyMesh::Point p = mesh.point(vhTmp);
    QVector3D vp = UtilsMesh::to_qvector3D(p);
    EdgeHandle eh;
    float min = static_cast<float>(INT_MAX);


    for (MyMesh::VertexEdgeCWIter ve_it = mesh.ve_cwiter(vhStart); ve_it.is_valid(); ve_it++)
    {
        EdgeHandle ehTmp = *ve_it;
        MyMesh::Point pTmp = mesh.calc_edge_midpoint(ehTmp);
        QVector3D vpTmp = UtilsMesh::to_qvector3D(pTmp);

        float dist = vpTmp.distanceToPoint(vp);
        if (min >= dist) {
            min = dist;
            eh = ehTmp;
        }
    }

    return eh;
}

void build_pathsOLD(int vertexNext, bool close)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    //    // Init
    //    EdgeHandle ehTmp = get_edge_seed(vertexSeed, vertexNext);
    //    edgeSeed = ehTmp.idx();

    //    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    //    costEdges[edgeSeed] = 0.0;
    //    vector<bool> edgesVisited(mesh.n_edges(), false);
    //    vector<int> activeList;   activeList.push_back(edgeSeed);
    //    paths = vector<int>(mesh.n_edges(), -1);

    //    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))
    //            || Utils::is_in_vector(criteres, static_cast<int>(STROKE_DIST))) {
    //        myDijkstra.calc_path(&mesh, vertexNext);
    //    }

    //    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    //    while (!activeList.empty())
    //    {
    //        int curEdge = get_minCostEdge_from_activeList(activeList, costEdges);
    //        Utils::erase_elt(activeList, curEdge);
    //        edgesVisited[curEdge] = true;

    //        // Voisinage
    //        vector<EdgeHandle> ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
    //        for (auto eh : ehs)
    //        {
    //            int edgeNeigh = eh.idx();

    //            // Si déjà visité
    //            if (edgesVisited[edgeNeigh])    continue;

    //            double tmpCost = costEdges[curEdge] + cost_function(edgeNeigh, close) ;

    //            // Voisin dans liste active ET  coût calculé inférieur au coût enregistré
    //            if (Utils::is_in_vector(activeList, edgeNeigh) &&  tmpCost < costEdges[edgeNeigh]) {
    //                Utils::erase_elt(activeList, edgeNeigh);
    //            }
    //            // Voisin pas dans la lsite active
    //            else if ( ! Utils::is_in_vector(activeList, edgeNeigh)) {
    //                costEdges[edgeNeigh] = tmpCost;
    //                paths[edgeNeigh] = curEdge;
    //                activeList.push_back(edgeNeigh);
    //            }
    //        }
    //    }

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

/*------------------------------------------------------------------------------
 * Construit les chemins pour le livewire (dans @paths).
 * @vertexNext pour le critère de visibilité avec dijkstra.
 * Pensez au cas où à bien mettre à jour @vertexSeed avant d'utiliser la fonction.
 * ----------------------------------------------------------------------------*/
void LiveWire::build_paths(int vertexNext, bool close)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    vector<double> costsToVertices(mesh.n_vertices(), static_cast<double>(INT_MAX));
    costsToVertices[vertexSeed] = 0.0;
    vector<bool> verticesVisited(mesh.n_vertices(), false);
    vector<int> activeList;   activeList.push_back(vertexSeed);
    paths = vector<int>(mesh.n_vertices(), -1);

    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))
            || Utils::is_in_vector(criteres, static_cast<int>(STROKE_DIST))) {
        myDijkstra.calc_path(&mesh, vertexNext);
    }

    while (!activeList.empty())
    {
        //        qDebug() << endl << endl << "\t\t\tcostsToVertices=" << costsToVertices;
        //        qDebug() << "\t\t\tverticesVisited=" << verticesVisited;
        //        qDebug() << "\t\t\tactiveList=" << activeList;
        //        qDebug() << "\t\t\tpaths=" << paths;

        int vertexCurrent = get_minCostToVertex_from_activeList(activeList, costsToVertices);
        VertexHandle vhCur;
        vhCur = mesh.vertex_handle(vertexCurrent);
        Utils::erase_elt(activeList, vertexCurrent);
        verticesVisited[vertexCurrent] = true;

        // Voisinage
        for (MyMesh::VertexEdgeCWIter ve_it = mesh.ve_cwiter(vhCur); ve_it.is_valid(); ve_it++)
        {
            EdgeHandle ehNeigh = *ve_it;
            int edgeNeigh = ehNeigh.idx();

            // On récupère l'autre sommet de l'arête
            VertexHandle vh1, vh2;
            UtilsMesh::get_vh_of_edge(&mesh, edgeNeigh, vh1, vh2);
            VertexHandle vhNeigh;
            if (vh1 == vhCur)   vhNeigh = vh2;
            else if (vh2 == vhCur)  vhNeigh = vh1;
            else {
                qWarning() << "error in" << __FUNCTION__ << ": vhNeigh not found";
            }
            int vertexNeigh = vhNeigh.idx();

            // Si déjà visité
            if (verticesVisited[vertexNeigh])    continue;

            double tmpCost = costsToVertices[vertexCurrent] + cost_function(vertexCurrent, edgeNeigh, close) ;

            // Voisin dans liste active ET  coût calculé inférieur au coût enregistré
            if (Utils::is_in_vector(activeList, vertexNeigh) &&  tmpCost < costsToVertices[vertexNeigh]) {
                Utils::erase_elt(activeList, vertexNeigh);
            }
            // Voisin pas dans la lsite active
            else if ( ! Utils::is_in_vector(activeList, vertexNeigh)) {
                costsToVertices[vertexNeigh] = tmpCost;
                // paths[edgeNeigh] = curEdge;
                paths[vertexNeigh] = vertexCurrent;
                activeList.push_back(vertexNeigh);
            }
        }
    }

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}

void LiveWire::build_paths_noEdgeSeed(int vertexNext, bool close)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    // Init

    vector<double> costEdges(mesh.n_edges(), static_cast<double>(INT_MAX));
    vector<bool> edgesVisited(mesh.n_edges(), false);
    vector<int> activeList;

    int first_edge = 0;
    int tmp = INT_MAX;
    for(auto ve = mesh.ve_iter(VertexHandle(vertexNext)) ; ve.is_valid() ; ve++)
    {

        int cost_tmp = cost_function(ve->idx(), close);
        if(cost_tmp < tmp)
        {
            first_edge = ve->idx();
            tmp = cost_tmp;
        }
    }

    edgeSeed = first_edge;
    costEdges[edgeSeed] = 0.0;

    activeList.push_back(edgeSeed);

    paths = vector<int>(mesh.n_edges(), -1);

    if (Utils::is_in_vector(criteres, static_cast<int>(VISIBILITY))
            || Utils::is_in_vector(criteres, static_cast<int>(STROKE_DIST))) {
        myDijkstra.calc_path(&mesh, vertexNext);
    }

    // WARNING --> BEGIN PAS INITIALISE DANS PATHS...

    while (!activeList.empty())
    {
        int curEdge = get_minCostToVertex_from_activeList(activeList, costEdges);
        Utils::erase_elt(activeList, curEdge);
        edgesVisited[curEdge] = true;

        // Voisinage
        vector<EdgeHandle> ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
        for (auto eh : ehs)
        {
            int edgeNeigh = eh.idx();

            // Si déjà visité
            if (edgesVisited[edgeNeigh])    continue;

            double tmpCost = costEdges[curEdge] + cost_function(edgeNeigh, close) ;

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
void Old_draw(unsigned vertex2)
{
    //    //    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    //    //    int red = Utils::randInt(0, 255);
    //    //    int blue = Utils::randInt(0, 255);
    //    //    int green = Utils::randInt(0, 255);
    //    int red = 255;
    //    int blue = 0;
    //    int green = 0;

    //    EdgeHandle eh1 = mesh.edge_handle(edgeSeed);
    //    EdgeHandle eh2 = get_edge_seed(vertex2, vertexSeed);
    //    unsigned curEdge = eh2.idx();
    //    vector<EdgeHandle> ehs;

    //    while (static_cast<int>(curEdge) != edgeSeed)
    //    {
    //        ehs.clear();
    //        ehs = UtilsMesh::get_edgeEdge_circulator(&mesh, curEdge);
    //        for (auto eh : ehs)
    //        {
    //            if (eh.idx()==paths[curEdge]) {
    //                mesh.set_color(eh, MyMesh::Color(red, green, blue));
    //                mesh.data(eh).thickness = 6;
    //            }
    //        }
    //        if (ehs.empty())    break;
    //        curEdge = paths[curEdge];
    //    }

    //    // point de départ et point d'arrivée en rouge et en gros
    //    mesh.set_color(eh1, MyMesh::Color(255, 0, 0));
    //    mesh.set_color(eh2, MyMesh::Color(255, 0, 0));
    //    mesh.data(eh1).thickness = 8;
    //    mesh.data(eh2).thickness = 8;
    //    VertexHandle vh1 = mesh.vertex_handle(vertexSeed);
    //    VertexHandle vh2 = mesh.vertex_handle(vertex2);
    //    mesh.set_color(vh1, MyMesh::Color(255, 0, 0));
    //    mesh.set_color(vh2, MyMesh::Color(255, 0, 0));
    //    mesh.data(vh1).thickness = 20;
    //    mesh.data(vh2).thickness = 20;

    //    //    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}


/*------------------------------------------------------------------------------
 * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
 * ----------------------------------------------------------------------------*/
void LiveWire::draw(unsigned vertex2)
{
    qDebug() << "\t\t<" << __FUNCTION__ << ">";

    //    int red = Utils::randInt(0, 255);
    //    int blue = Utils::randInt(0, 255);
    //    int green = Utils::randInt(0, 255);
    int red = 255;
    int blue = 0;
    int green = 0;


    unsigned vertexCurrent = vertex2;
    VertexHandle vhCur = mesh.vertex_handle(vertexCurrent);

    while (static_cast<int>(vertexCurrent) != vertexSeed)
    {

        bool validator=false;
        for (MyMesh::VertexEdgeCWIter ve_it = mesh.ve_cwiter(vhCur); ve_it.is_valid(); ve_it++)
        {

            EdgeHandle ehCur = *ve_it;
            int edgeCurrent = ehCur.idx();

            VertexHandle vh1, vh2;
            UtilsMesh::get_vh_of_edge(&mesh, edgeCurrent, vh1, vh2);
            VertexHandle vhNeigh;
            if (vh1 == vhCur)   vhNeigh = vh2;
            else if (vh2 == vhCur)  vhNeigh = vh1;
            else {
                qWarning() << "error in" << __FUNCTION__ << ": vhNeigh not found";
            }
            int vertexNeigh = vhNeigh.idx();

            if (vertexNeigh==paths[vertexCurrent]) {
                mesh.set_color(ehCur, MyMesh::Color(red, green, blue));
                mesh.data(ehCur).thickness = 6;
                validator=true;
                break;
            }
            validator=true;
        }
        if (validator) {
            vertexCurrent = paths[vertexCurrent];
            vhCur = mesh.vertex_handle(vertexCurrent);
        }
    }

    // point de départ et point d'arrivée en rouge et en gros
    VertexHandle vhSeed = mesh.vertex_handle(vertexSeed);
    VertexHandle vhEnd = mesh.vertex_handle(vertex2);
    mesh.set_color(vhSeed, MyMesh::Color(255, 0, 0));
    mesh.set_color(vhEnd, MyMesh::Color(255, 0, 0));
    mesh.data(vhSeed).thickness = 20;
    mesh.data(vhEnd).thickness = 20;

    qDebug() << "\t\t</" << __FUNCTION__ << ">";
}













