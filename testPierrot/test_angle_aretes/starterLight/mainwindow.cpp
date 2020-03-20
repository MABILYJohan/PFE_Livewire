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

/*float MainWindow::angleEE(MyMesh* _mesh, int vertexID, int faceID)
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
}*/

void get_vh_of_edge(MyMesh *_mesh, int edgeID,
                               VertexHandle &vh0, VertexHandle &vh1)
{
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    MyMesh::HalfedgeHandle heh1 =  _mesh->halfedge_handle(eh, 0);
    //    MyMesh::HalfedgeHandle heh2 =  _mesh->halfedge_handle(eh, 1);
    vh0 = _mesh->from_vertex_handle(heh1);
    vh1 = _mesh->to_vertex_handle(heh1);
}


/*** FONCTION OK ***/
float MainWindow::angleEE(MyMesh* _mesh, int vertexID, int edge0, int edge1)
{
    QVector<QVector3D> points;
    VertexHandle vhID=_mesh->vertex_handle(vertexID);
    QVector3D point_origine;
    point_origine = QVector3D(_mesh->point(vhID)[0],_mesh->point(vhID)[1],_mesh->point(vhID)[2]);

    VertexHandle vh0_edge0, vh1_edge0, vh0_edge1, vh1_edge1;
    get_vh_of_edge(_mesh, edge0, vh0_edge0, vh1_edge0);
    get_vh_of_edge(_mesh, edge1, vh0_edge1, vh1_edge1);

    if (vh0_edge0.idx()!=vertexID) points.push_back(QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]));
    else points.push_back(QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]));

    if (vh0_edge1.idx()!=vertexID) points.push_back(QVector3D(_mesh->point(vh0_edge1)[0],_mesh->point(vh0_edge1)[1],_mesh->point(vh0_edge1)[2]));
    else points.push_back(QVector3D(_mesh->point(vh1_edge1)[0],_mesh->point(vh1_edge1)[1],_mesh->point(vh1_edge1)[2]));

    QVector3D vecteur1 = points[1] - point_origine;
    QVector3D vecteur2 = points[0] - point_origine;
    vecteur1.normalize();
    vecteur2.normalize();

    /*qDebug() << point_origine;
    qDebug() << "choix : " << QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]);
    qDebug() << "et : " << QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]);
    qDebug() << "choisi : " <<points[0];
    qDebug() << "choix : " << QVector3D(_mesh->point(vh0_edge1)[0],_mesh->point(vh0_edge1)[1],_mesh->point(vh0_edge1)[2]);
    qDebug() << "et : " << QVector3D(_mesh->point(vh1_edge1)[0],_mesh->point(vh1_edge1)[1],_mesh->point(vh1_edge1)[2]);
    qDebug() << "choisi : " << points[1];*/


    return acos(QVector3D::dotProduct(vecteur1, vecteur2));
}

double MainWindow::critere_angleEE(MyMesh* _mesh, EdgeHandle eh0, EdgeHandle eh1)
{
    QVector<QVector3D> points;
    QVector3D point_origine;

    VertexHandle vh0_edge0, vh1_edge0, vh0_edge1, vh1_edge1;
    get_vh_of_edge(_mesh, eh0.idx(), vh0_edge0, vh1_edge0);
    get_vh_of_edge(_mesh, eh1.idx(), vh0_edge1, vh1_edge1);

    try
    {
        if (vh0_edge0.idx()==vh0_edge1.idx())
        {
            point_origine = QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]);
            points.push_back(QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]));
            points.push_back(QVector3D(_mesh->point(vh1_edge1)[0],_mesh->point(vh1_edge1)[1],_mesh->point(vh1_edge1)[2]));
        }
        else if (vh0_edge0.idx()==vh1_edge1.idx())
        {
            point_origine = QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]);
            points.push_back(QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]));
            points.push_back(QVector3D(_mesh->point(vh0_edge1)[0],_mesh->point(vh0_edge1)[1],_mesh->point(vh0_edge1)[2]));
        }
        else if (vh1_edge0.idx()==vh0_edge1.idx())
        {
            point_origine = QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]);
            points.push_back(QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]));
            points.push_back(QVector3D(_mesh->point(vh1_edge1)[0],_mesh->point(vh1_edge1)[1],_mesh->point(vh1_edge1)[2]));
        }
        else if (vh1_edge0.idx()==vh1_edge1.idx())
        {
            point_origine = QVector3D(_mesh->point(vh1_edge0)[0],_mesh->point(vh1_edge0)[1],_mesh->point(vh1_edge0)[2]);
            points.push_back(QVector3D(_mesh->point(vh0_edge0)[0],_mesh->point(vh0_edge0)[1],_mesh->point(vh0_edge0)[2]));
            points.push_back(QVector3D(_mesh->point(vh0_edge1)[0],_mesh->point(vh0_edge1)[1],_mesh->point(vh0_edge1)[2]));
        }
        else throw std::string("aretes non adjacentes!");
    }
    catch(std::string const& err) {std::cerr << err << std::endl;}

    QVector3D vecteur1 = points[1] - point_origine;
    QVector3D vecteur2 = points[0] - point_origine;
    vecteur1.normalize();
    vecteur2.normalize();

    return acos(QVector3D::dotProduct(vecteur1, vecteur2));
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
    for (auto e : tabcurv) e=(e-minE)/(maxE-minE);
    qDebug() << mesh.n_edges();*/
    EdgeHandle eh0 = mesh.edge_handle(0);
    EdgeHandle eh1 = mesh.edge_handle(7);
    qDebug() << "angle : " << critere_angleEE(&mesh,eh0,eh1);
    VertexHandle vh = mesh.vertex_handle(0);
    mesh.set_color(vh, MyMesh::Color(255, 255, 0));
    mesh.data(vh).thickness = 15;
    mesh.set_color(eh0, MyMesh::Color(0, 255, 0));
    mesh.data(eh0).thickness = 8;
    mesh.set_color(eh1, MyMesh::Color(0, 255, 0));
    mesh.data(eh1).thickness = 8;
    displayMesh(&mesh);
}



