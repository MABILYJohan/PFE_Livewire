#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

vector<unsigned> MainWindow::test_contour(vector<MyMesh::Point> points, int precision)
{
    vector<unsigned> tmpVertices;
    int vertexID;
    for (auto p : points)
    {
        vertexID = UtilsMesh::get_vertex_of_point(&mesh, p, precision);
        if (vertexID >= 0)  tmpVertices.push_back(vertexID);
        else qDebug() << "point P(" << p[0]<<","<<p[1]<<","<<p[2]<<") not found";
    }
    return tmpVertices;
}

/*------------------------------------------------------------------------------
 * Fonction utilitaire pour choisir des sommets à utiliser pour le contour
 * selon une liste de points.
 * ----------------------------------------------------------------------------*/
vector<unsigned> MainWindow::get_verticesID(int _testChoice)
{
    vector<unsigned> tmpVertices;

    vector<MyMesh::Point> tmpPoints;

    switch(_testChoice)
    {
    //////////////////  BUNNY   /////////////////////
    case(1):
        tmpPoints.push_back(MyMesh::Point(-0.288f, 0.082f, 0.148f));
        tmpPoints.push_back(MyMesh::Point(-0.105f, 0.133f, 0.133f));
        tmpPoints.push_back(MyMesh::Point(-0.069f, 0.201f, 0.001f));
        tmpPoints.push_back(MyMesh::Point(-0.149f, 0.174f, -0.095f));
        tmpPoints.push_back(MyMesh::Point(-0.313f, 0.085f, -0.031f));
        break;
    case(2):
        tmpPoints.push_back(MyMesh::Point(-0.263f, 0.075f, 0.165f));
        tmpPoints.push_back(MyMesh::Point(-0.105f, 0.133f, 0.133f));
        tmpPoints.push_back(MyMesh::Point(-0.069f, 0.201f, 0.001f));
        break;
    case(3):
        tmpPoints.push_back(MyMesh::Point(0.139f, 0.029f, 0.227f));
        tmpPoints.push_back(MyMesh::Point(0.240f, 0.001f, 0.247f));
        tmpPoints.push_back(MyMesh::Point(0.289f, -0.092f, 0.258f));
        tmpPoints.push_back(MyMesh::Point(0.269f, -0.199f, 0.244f));
        tmpPoints.push_back(MyMesh::Point(0.022f, -0.129f, 0.198f));
        break;
    case(4):
        tmpPoints.push_back(MyMesh::Point(-0.167f, 0.360f, -0.006f));
        tmpPoints.push_back(MyMesh::Point(-0.107f, 0.281f, -0.000f));
        break;
    case(5):
        tmpPoints.push_back(MyMesh::Point(0.464f, -0.126f, 0.019f));
        tmpPoints.push_back(MyMesh::Point(0.424f, -0.198f, 0.122f));
        tmpPoints.push_back(MyMesh::Point(0.355f, -0.330f, 0.107f));
        break;

    //////////////////  AUTRES   /////////////////////

    default:
        qDebug() << "TEST NOT DEFINED";
        break;
    }

    tmpVertices = test_contour(tmpPoints, 1);

    return tmpVertices;
}

MyMesh::Point MainWindow::get_pt_de_vue()
{
    qDebug() << "<" << __FUNCTION__ << ">";
    ui->displayWidget->my_view();
    OpenMesh::Vec3f center3F = ui->displayWidget->myCenter;
    QVector3D center (center3F[0], center3F[1], center3F[2]);

    for (MyMesh::VertexIter curVert = mesh.vertices_begin(); curVert != mesh.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        MyMesh::Point P = mesh.point(vh);
        if (P[2] > center.z()) {
            center.setZ(P[2]+2.f);
        }
    }

    qDebug() << "\tcenter = " << center;
    qDebug() << "</" << __FUNCTION__ << ">";
    return (MyMesh::Point(center.x(), center.y(), center.z()));
}

void MainWindow::make_livewire()
{
    qDebug() << "<" << __FUNCTION__ << ">";

    mesh.update_normals();
    // initialisation des couleurs et épaisseurs (sommets et arêtes) du mesh
    resetAllColorsAndThickness(&mesh);

    UtilsMesh::extract_biggest_connexity_component(&mesh);

    int nbConComp = UtilsMesh::nb_connexity_componenents(&mesh);
    qDebug() << "nb de composantes connexes =" << nbConComp;


    //    vector<unsigned> tmpVertices = {0, 15, 65, 75};

    //    vector<unsigned> tmpVertices = get_verticesID(testChoice);
    //    Contour myContour(mesh, tmpVertices);

    //    char path[70] = {"../donneesPFE M2GIG/MySon/Test/Contour/contour_visibleVersion.obj\0"};
    //    char path[80] = {"../masks3D_Myson_02_contour.obj\0"};
    //    char path[80] = {"../../donneesPFE M2GIG/Siva/Test/mask_048_1_contour.xyz\0"};
    char path[80] = {"../maks3D_Myson_02.xyz\0"};
    Contour myContour(mesh, path);
    myContour.reduct(3);

    myContour.display(1, true);

    MyMesh::Point _sightPoint = get_pt_de_vue();

    myContour.draw_contour(&mesh, _sightPoint);

    // on affiche le maillage
    displayMesh(&mesh);

    qDebug() << "</" << __FUNCTION__ << ">";
}

/*-------------------------------------------------------------------------
 * @displayDist valeur pour modifier la distance d'affichage z des points
 * pour ne pas qu'ils se confondent sur le maillage
 * -----------------------------------------------------------------------*/
void MainWindow::vizuContour(int displayDist)
{
    char path[80] = {"../maks3D_Myson_02.obj\0"};
    MyMesh myMeshContour;
    OpenMesh::IO::read_mesh(myMeshContour, path);
    vector<MyMesh::Point> myPoints;
    for (MyMesh::VertexIter curVert = myMeshContour.vertices_begin(); curVert != myMeshContour.vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        MyMesh::Point p = myMeshContour.point(vh);
        myPoints.push_back(p);
    }
    for (auto p : myPoints)
    {
        p[2] += displayDist*0.01f;
        VertexHandle vh = mesh.add_vertex(p);
        mesh.data(vh).thickness = 8;
        mesh.set_color(vh, MyMesh::Color(0, 0, 255));
    }
    displayMesh(&mesh);
}

/* **** début de la partie boutons et IHM **** */

void MainWindow::on_pushButton_livewire_clicked()
{
    qDebug() <<"<" << __FUNCTION__ << "The event sender is" << sender() << ">";

    make_livewire();

    qDebug() << "</" << __FUNCTION__ << ">";
}

void MainWindow::on_pushButton_vizuContour_clicked()
{
    qDebug() <<"<" << __FUNCTION__ << "The event sender is" << sender() << ">";
    vizuContour(0);
    qDebug() << "</" << __FUNCTION__ << ">";
}

void MainWindow::on_spinBox_valueChanged(int value)
{
    qDebug() <<"<" << __FUNCTION__ << "The event sender is" << sender() << ">";
    testChoice = value;
    qDebug() <<"\ttestChoice =" << testChoice;
    qDebug() << "</" << __FUNCTION__ << ">";
}

// exemple pour charger un fichier .obj
void MainWindow::on_pushButton_chargement_clicked()
{
    // fenêtre de sélection des fichiers
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Mesh"),
                                                    // "../../../MGM/meshFiles/",
                                                    "../../donneesPFE M2GIG/Siva/Test/",
                                                    // "../../donneesPFE M2GIG/MySon/Test/",
                                                    tr("Mesh Files (*.obj)"));

    // chargement du fichier .obj dans la variable globale "mesh"
    OpenMesh::IO::read_mesh(mesh, fileName.toUtf8().constData());

    mesh.update_normals();

    // initialisation des couleurs et épaisseurs (sommets et arêtes) du mesh
    resetAllColorsAndThickness(&mesh);

    // TMP
    mesh.triangulate();
    mesh.update_normals();
    resetAllColorsAndThickness(&mesh);

    // on affiche le maillage
    displayMesh(&mesh);
}

// exemple pour construire un mesh face par face
void MainWindow::on_pushButton_generer_clicked()
{

    // on construit une liste de sommets
    MyMesh::VertexHandle sommets[8];
    sommets[0] = mesh.add_vertex(MyMesh::Point(-1, -1,  1));
    sommets[1] = mesh.add_vertex(MyMesh::Point( 1, -1,  1));
    sommets[2] = mesh.add_vertex(MyMesh::Point( 1,  1,  1));
    sommets[3] = mesh.add_vertex(MyMesh::Point(-1,  1,  1));
    sommets[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
    sommets[5] = mesh.add_vertex(MyMesh::Point( 1, -1, -1));
    sommets[6] = mesh.add_vertex(MyMesh::Point( 1,  1, -1));
    sommets[7] = mesh.add_vertex(MyMesh::Point(-1,  1, -1));


    // on construit des faces à partir des sommets

    std::vector<MyMesh::VertexHandle> uneNouvelleFace;

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[0]);
    uneNouvelleFace.push_back(sommets[1]);
    uneNouvelleFace.push_back(sommets[2]);
    uneNouvelleFace.push_back(sommets[3]);
    mesh.add_face(uneNouvelleFace);

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[7]);
    uneNouvelleFace.push_back(sommets[6]);
    uneNouvelleFace.push_back(sommets[5]);
    uneNouvelleFace.push_back(sommets[4]);
    mesh.add_face(uneNouvelleFace);

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[1]);
    uneNouvelleFace.push_back(sommets[0]);
    uneNouvelleFace.push_back(sommets[4]);
    uneNouvelleFace.push_back(sommets[5]);
    mesh.add_face(uneNouvelleFace);

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[2]);
    uneNouvelleFace.push_back(sommets[1]);
    uneNouvelleFace.push_back(sommets[5]);
    uneNouvelleFace.push_back(sommets[6]);
    mesh.add_face(uneNouvelleFace);

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[3]);
    uneNouvelleFace.push_back(sommets[2]);
    uneNouvelleFace.push_back(sommets[6]);
    uneNouvelleFace.push_back(sommets[7]);
    mesh.add_face(uneNouvelleFace);

    uneNouvelleFace.clear();
    uneNouvelleFace.push_back(sommets[0]);
    uneNouvelleFace.push_back(sommets[3]);
    uneNouvelleFace.push_back(sommets[7]);
    uneNouvelleFace.push_back(sommets[4]);
    mesh.add_face(uneNouvelleFace);

    mesh.update_normals();

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
void MainWindow::displayMesh(MyMesh* _mesh, bool isTemperatureMap, float mapRange)
{
    GLuint* triIndiceArray = new GLuint[_mesh->n_faces() * 3];
    GLfloat* triCols = new GLfloat[_mesh->n_faces() * 3 * 3];
    GLfloat* triVerts = new GLfloat[_mesh->n_faces() * 3 * 3];

    int i = 0;

    if(isTemperatureMap)
    {
        QVector<float> values;

        if(mapRange == -1)
        {
            for (MyMesh::VertexIter curVert = _mesh->vertices_begin(); curVert != _mesh->vertices_end(); curVert++)
                values.append(fabs(_mesh->data(*curVert).value));
            qSort(values);
            mapRange = values.at(values.size()*0.8);
            qDebug() << "mapRange" << mapRange;
        }

        float range = mapRange;
        MyMesh::ConstFaceIter fIt(_mesh->faces_begin()), fEnd(_mesh->faces_end());
        MyMesh::ConstFaceVertexIter fvIt;

        for (; fIt!=fEnd; ++fIt)
        {
            fvIt = _mesh->cfv_iter(*fIt);
            if(_mesh->data(*fvIt).value > 0){triCols[3*i+0] = 255; triCols[3*i+1] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+2] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            else{triCols[3*i+2] = 255; triCols[3*i+1] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+0] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
            triIndiceArray[i] = i;

            i++; ++fvIt;
            if(_mesh->data(*fvIt).value > 0){triCols[3*i+0] = 255; triCols[3*i+1] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+2] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            else{triCols[3*i+2] = 255; triCols[3*i+1] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+0] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
            triIndiceArray[i] = i;

            i++; ++fvIt;
            if(_mesh->data(*fvIt).value > 0){triCols[3*i+0] = 255; triCols[3*i+1] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+2] = 255 - std::min((_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            else{triCols[3*i+2] = 255; triCols[3*i+1] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0); triCols[3*i+0] = 255 - std::min((-_mesh->data(*fvIt).value/range) * 255.0, 255.0);}
            triVerts[3*i+0] = _mesh->point(*fvIt)[0]; triVerts[3*i+1] = _mesh->point(*fvIt)[1]; triVerts[3*i+2] = _mesh->point(*fvIt)[2];
            triIndiceArray[i] = i;

            i++;
        }
    }
    else
    {
        MyMesh::ConstFaceIter fIt(_mesh->faces_begin()), fEnd(_mesh->faces_end());
        MyMesh::ConstFaceVertexIter fvIt;
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




