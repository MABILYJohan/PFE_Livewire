#include "mainwindow.h"
#include "ui_mainwindow.h"


/* **** début de la partie boutons et IHM **** */


void MainWindow::on_pushButton_chargement_clicked()
{
    // fenêtre de sélection des fichiers
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Mesh"), "", tr("Mesh Files (*.obj)"));

    // chargement du fichier .obj dans la variable globale "mesh"
    OpenMesh::IO::read_mesh(mesh, fileName.toUtf8().constData());

    // initialisation des couleurs et épaisseurs (sommets et arêtes) du mesh
    resetAllColorsAndThickness(&mesh);

    // on affiche le maillage
    displayMesh(&mesh);
}
/* **** fin de la partie boutons et IHM **** */



/* **** fonctions supplémentaires **** */
// permet d'initialiser les couleurs et les épaisseurs des élements du maillage
void MainWindow::resetAllColorsAndThickness(MyMesh* _mesh)
{
    for (MyMesh::VertexIter curVert = _mesh->vertices_begin(); curVert != _mesh->vertices_end(); curVert++)
    {
        _mesh->data(*curVert).thickness = 1;
        _mesh->set_color(*curVert, MyMesh::Color(0, 0, 0));
    }

    for (MyMesh::FaceIter curFace = _mesh->faces_begin(); curFace != _mesh->faces_end(); curFace++)
    {
        _mesh->set_color(*curFace, MyMesh::Color(150, 150, 150));
    }

    for (MyMesh::EdgeIter curEdge = _mesh->edges_begin(); curEdge != _mesh->edges_end(); curEdge++)
    {
        _mesh->data(*curEdge).thickness = 1;
        _mesh->set_color(*curEdge, MyMesh::Color(0, 0, 0));
    }
}

// charge un objet MyMesh dans l'environnement OpenGL
void MainWindow::displayMesh(MyMesh* _mesh)
{
    GLuint* triIndiceArray = new GLuint[_mesh->n_faces() * 3];
    GLfloat* triCols = new GLfloat[_mesh->n_faces() * 3 * 3];
    GLfloat* triVerts = new GLfloat[_mesh->n_faces() * 3 * 3];

    MyMesh::ConstFaceIter fIt(_mesh->faces_begin()), fEnd(_mesh->faces_end());
    MyMesh::ConstFaceVertexIter fvIt;
    int i = 0;
    for (; fIt!=fEnd; ++fIt)
    {
        fvIt = _mesh->cfv_iter(*fIt);
        triCols[3*i+0] = _mesh->color(*fIt)[0]; triCols[3*i+1] = _mesh->color(*fIt)[1]; triCols[3*i+2] = _mesh->color(*fIt)[2];
        triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
        triIndiceArray[i] = i;

        i++; ++fvIt;
        triCols[3*i+0] = _mesh->color(*fIt)[0]; triCols[3*i+1] = _mesh->color(*fIt)[1]; triCols[3*i+2] = _mesh->color(*fIt)[2];
        triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
        triIndiceArray[i] = i;

        i++; ++fvIt;
        triCols[3*i+0] = _mesh->color(*fIt)[0]; triCols[3*i+1] = _mesh->color(*fIt)[1]; triCols[3*i+2] = _mesh->color(*fIt)[2];
        triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
        triIndiceArray[i] = i;

        i++;
    }

    ui->displayWidget->loadMesh(triVerts, triCols, _mesh->n_faces() * 3 * 3, triIndiceArray, _mesh->n_faces() * 3);

    delete[] triIndiceArray;
    delete[] triCols;
    delete[] triVerts;

    GLuint* linesIndiceArray = new GLuint[_mesh->n_edges() * 2];
    GLfloat* linesCols = new GLfloat[_mesh->n_edges() * 2 * 3];
    GLfloat* linesVerts = new GLfloat[_mesh->n_edges() * 2 * 3];

    i = 0;
    QHash<float, QList<int> > edgesIDbyThickness;
    for (MyMesh::EdgeIter eit = _mesh->edges_begin(); eit != _mesh->edges_end(); ++eit)
    {
        float t = _mesh->data(*eit).thickness;
        if(t > 0)
        {
            if(!edgesIDbyThickness.contains(t))
                edgesIDbyThickness[t] = QList<int>();
            edgesIDbyThickness[t].append((*eit).idx());
        }
    }
    QHashIterator<float, QList<int> > it(edgesIDbyThickness);
    QList<QPair<float, int> > edgeSizes;
    while (it.hasNext())
    {
        it.next();

        for(int e = 0; e < it.value().size(); e++)
        {
            int eidx = it.value().at(e);

            MyMesh::VertexHandle vh1 = _mesh->to_vertex_handle(_mesh->halfedge_handle(_mesh->edge_handle(eidx), 0));
            linesVerts[3*i+0] = _mesh->point(vh1)[0];
            linesVerts[3*i+1] = _mesh->point(vh1)[1];
            linesVerts[3*i+2] = _mesh->point(vh1)[2];
            linesCols[3*i+0] = _mesh->color(_mesh->edge_handle(eidx))[0];
            linesCols[3*i+1] = _mesh->color(_mesh->edge_handle(eidx))[1];
            linesCols[3*i+2] = _mesh->color(_mesh->edge_handle(eidx))[2];
            linesIndiceArray[i] = i;
            i++;

            MyMesh::VertexHandle vh2 = _mesh->from_vertex_handle(_mesh->halfedge_handle(_mesh->edge_handle(eidx), 0));
            linesVerts[3*i+0] = _mesh->point(vh2)[0];
            linesVerts[3*i+1] = _mesh->point(vh2)[1];
            linesVerts[3*i+2] = _mesh->point(vh2)[2];
            linesCols[3*i+0] = _mesh->color(_mesh->edge_handle(eidx))[0];
            linesCols[3*i+1] = _mesh->color(_mesh->edge_handle(eidx))[1];
            linesCols[3*i+2] = _mesh->color(_mesh->edge_handle(eidx))[2];
            linesIndiceArray[i] = i;
            i++;
        }
        edgeSizes.append(qMakePair(it.key(), it.value().size()));
    }

    ui->displayWidget->loadLines(linesVerts, linesCols, i * 3, linesIndiceArray, i, edgeSizes);

    delete[] linesIndiceArray;
    delete[] linesCols;
    delete[] linesVerts;

    GLuint* pointsIndiceArray = new GLuint[_mesh->n_vertices()];
    GLfloat* pointsCols = new GLfloat[_mesh->n_vertices() * 3];
    GLfloat* pointsVerts = new GLfloat[_mesh->n_vertices() * 3];

    i = 0;
    QHash<float, QList<int> > vertsIDbyThickness;
    for (MyMesh::VertexIter vit = _mesh->vertices_begin(); vit != _mesh->vertices_end(); ++vit)
    {
        float t = _mesh->data(*vit).thickness;
        if(t > 0)
        {
            if(!vertsIDbyThickness.contains(t))
                vertsIDbyThickness[t] = QList<int>();
            vertsIDbyThickness[t].append((*vit).idx());
        }
    }
    QHashIterator<float, QList<int> > vitt(vertsIDbyThickness);
    QList<QPair<float, int> > vertsSizes;

    while (vitt.hasNext())
    {
        vitt.next();

        for(int v = 0; v < vitt.value().size(); v++)
        {
            int vidx = vitt.value().at(v);

            pointsVerts[3*i+0] = _mesh->point(_mesh->vertex_handle(vidx))[0];
            pointsVerts[3*i+1] = _mesh->point(_mesh->vertex_handle(vidx))[1];
            pointsVerts[3*i+2] = _mesh->point(_mesh->vertex_handle(vidx))[2];
            pointsCols[3*i+0] = _mesh->color(_mesh->vertex_handle(vidx))[0];
            pointsCols[3*i+1] = _mesh->color(_mesh->vertex_handle(vidx))[1];
            pointsCols[3*i+2] = _mesh->color(_mesh->vertex_handle(vidx))[2];
            pointsIndiceArray[i] = i;
            i++;
        }
        vertsSizes.append(qMakePair(vitt.key(), vitt.value().size()));
    }

    ui->displayWidget->loadPoints(pointsVerts, pointsCols, i * 3, pointsIndiceArray, i, vertsSizes);

    delete[] pointsIndiceArray;
    delete[] pointsCols;
    delete[] pointsVerts;
}


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    vertexSelection = -1;
    edgeSelection = -1;
    faceSelection = -1;

    modevoisinage = false;

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

/*---------------------------------------
 * Fonctions pour la courbure gaussienne
 * --------------------------------------*/

//float MainWindow::angleEE(MyMesh* _mesh, int vertexID,  int faceID)
//{
//    MyMesh::Point v1;
//    MyMesh::Point v2;
//    FaceHandle fhId = _mesh->face_handle(faceID);
//    VertexHandle vhId = _mesh->vertex_handle(vertexID);
//    std::vector<VertexHandle> vh;
//    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fhId); fv_it.is_valid(); fv_it++)
//    {
//        vh.push_back(*fv_it);
//    }

//    /*MyMesh::Point A;
//    MyMesh::Point B;
//    MyMesh::Point C;*/
//    QVector3D A;
//    QVector3D B;
//    QVector3D C;
//    for (unsigned i=0; i<vh.size(); i++)
//    {
//        if (vh[i] == vhId) {
//            A = _mesh->point (vh[i]);
//            int k=i+1;
//            if (k>=vh.size()) k=0;
//            B = _mesh->point (vh[k]);
//            k++;
//            if (k>=vh.size()) k=0;
//            C = _mesh->point (vh[k]);
//            break;
//        }
//    }
//    v1 = B-A;
//    v2 = C-A;
//    v1.normalize();
//    v2.normalize();

//    float angle = acos((v1 | v2));

//    return angle;
//}

float MainWindow::angleEE(MyMesh* _mesh, int vertexID, int faceID)
{
//    FaceHandle face_h = _mesh->face_handle(faceID);
//    QVector<MyMesh::Point> points;
//    MyMesh::Point point_origine;

//    // on cherche le point d'origine
//    for(MyMesh::FaceVertexIter curVer = _mesh->fv_begin(face_h); curVer.is_valid(); curVer++) {
//        VertexHandle vertex_h = *curVer;

//        if(vertex_h.idx() == vertexID) {
//            point_origine = _mesh->point(vertex_h);
//        }
//        else {
//            points.push_back(_mesh->point(vertex_h));
//        }
//    }

//    MyMesh::Point vecteur1 = points[1] - point_origine;
//    MyMesh::Point vecteur2 = points[0] - point_origine;
//    vecteur1.normalize();
//    vecteur2.normalize();

//    return acos(dot(vecteur1, vecteur2));

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

float MainWindow::faceArea(MyMesh* _mesh, int faceID)
{

    FaceHandle face_h = FaceHandle(faceID);
    QVector<MyMesh::Point> points;
    for(MyMesh::FaceVertexIter curVer = _mesh->fv_iter(face_h); curVer.is_valid(); curVer++) {
        VertexHandle vertex_h = *curVer;
        points.push_back(_mesh->point(vertex_h));
    }

    float aire = norm((points[1] - points[0]) % (points[2] - points[0])) / 2;
    return aire;
}

float MainWindow::aire_barycentrique(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float area = 0;
    for(MyMesh::VertexFaceIter vfit = _mesh->vf_iter(vh); vfit.is_valid(); vfit++){
        area += faceArea(_mesh,(*vfit).idx());
    }
    return area / 3;
}

double MainWindow::K_Curv(MyMesh* _mesh, int vertID)
{
    VertexHandle vh = _mesh->vertex_handle(vertID);
    float a = 1 / aire_barycentrique(_mesh, vh.idx());
    //qDebug() << "a : " << a;
    float theta = 0.f;
    for (MyMesh::VertexFaceCWIter vf_it = _mesh->vf_cwiter(vh); vf_it.is_valid(); vf_it++)
        {
            FaceHandle fh = *vf_it;
            theta += angleEE(_mesh, vh.idx(), fh.idx());
            //qDebug() << "theta : " << theta;
        }
    float b = 2*M_PI- theta;
    //qDebug() << "b : " << b;
    float K = a*b;
    //qDebug() << "sommet " << vertID << " : " << K;
    return K;
}

void get_vh_of_edge(MyMesh mesh, int edgeID,
                               VertexHandle &vh0, VertexHandle &vh1)
{
    EdgeHandle eh = mesh.edge_handle(edgeID);
    MyMesh::HalfedgeHandle heh1 =  mesh.halfedge_handle(eh, 0);
    //    MyMesh::HalfedgeHandle heh2 =  _mesh->halfedge_handle(eh, 1);
    vh0 = mesh.from_vertex_handle(heh1);
    vh1 = mesh.to_vertex_handle(heh1);
}

void MainWindow::on_pushButton_clicked()
{
    /*double minE,maxE;
    for (MyMesh::VertexIter curVert = mesh.vertices_begin(); curVert != mesh.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        tabcurv.push_back(abs(K_Curv(&mesh, vh.idx())));
    }
    minE = *std::min_element(tabcurv.begin(), tabcurv.end());
    maxE = *std::max_element(tabcurv.begin(), tabcurv.end());
    for (auto e : tabcurv) e=(e-minE)/(maxE-minE);*/
    std::vector<int> ehs(6);
    ehs.push_back(25245);
    ehs.push_back(25246);
    ehs.push_back(25247);
    ehs.push_back(25249);
    ehs.push_back(25250);
    ehs.push_back(25241);
    for (auto e : ehs)
    {
        /*EdgeHandle eh = mesh.edge_handle(e);
        mesh.set_color(eh, MyMesh::Color(255, 255, 0));
        mesh.data(eh).thickness = 5;*/
        VertexHandle vh0, vh1;
        get_vh_of_edge(mesh, e, vh0,vh1);
        qDebug() << "K_commun " << vh0.idx() << " : " << K_Curv(&mesh, vh0.idx());
        qDebug() << "K_suivant " << vh1.idx() << " : " << K_Curv(&mesh, vh1.idx());

        qDebug() << "arete " << e << " : " << K_Curv(&mesh, vh0.idx())+K_Curv(&mesh, vh1.idx())/2.0;
    }

}



