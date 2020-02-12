
// TP3 MGM

#include "mainwindow.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <vector>
#include <cmath>
#include <QVector3D>


using namespace std;

float MainWindow::faceArea(MyMesh* _mesh, int faceID)
{
//    qDebug() << "<" << __FUNCTION__ << ">";

    FaceHandle face_h = FaceHandle(faceID);

    // on enregistre les points de la face dans un QVector
    QVector<MyMesh::Point> points;
    for(MyMesh::FaceVertexIter curVer = _mesh->fv_iter(face_h); curVer.is_valid(); curVer++) {
        VertexHandle vertex_h = *curVer;
        points.push_back(_mesh->point(vertex_h));
    }

    float aire = norm((points[1] - points[0]) % (points[2] - points[0])) / 2;
//    qDebug() << "aire face" << faceID << " = " << aire ;
//    qDebug() << "</" << __FUNCTION__ << ">";
    return aire;
}

float MainWindow::angleFF(MyMesh* _mesh, int faceID0,  int faceID1, int vertID0, int vertID1)
{
    // calcul des normales des faces que l'on normalise
    FaceHandle fh1 = _mesh->face_handle(faceID0);
    FaceHandle fh2 = _mesh->face_handle(faceID1);
    MyMesh::Normal normal1 = _mesh->calc_face_normal(fh1);
    MyMesh::Normal normal2 = _mesh->calc_face_normal(fh2);
    normal1.normalize();
    normal2.normalize();

    MyMesh::Point point1, point2;
    point1 = _mesh->point(_mesh->vertex_handle(vertID0));
    point2 = _mesh->point(_mesh->vertex_handle(vertID1));

    // calcul du vecteur entre point 1 et 2
    MyMesh::Normal vector_v1_v2 = point2 - point1;

    // calcul de l'angle entre les deux normales des faces 1 et 2
    float angle = acos(dot(normal1, normal2));

    //qDebug() << "(" << faceID0 << ", " << faceID1 << ", " << vertID0 << ", " << vertID1 << ") : " << angle * 180 / PI << endl;
    //qDebug() << dot(cross_product, vector_v1_v2) << endl;

    // determiner le signe de l'angle
    MyMesh::Normal cross_product = cross(normal1, normal2);
    return dot(cross_product, vector_v1_v2) < 0 ? - angle : angle;
}

float MainWindow::angleEE(MyMesh* _mesh, int vertexID,  int faceID)
{
    /* **** à compléter ! **** */

    MyMesh::Point v1;
    MyMesh::Point v2;
    FaceHandle fhId = _mesh->face_handle(faceID);
    VertexHandle vhId = _mesh->vertex_handle(vertexID);
    std::vector<VertexHandle> vh;
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fhId); fv_it.is_valid(); fv_it++)
    {
        vh.push_back(*fv_it);
    }

    MyMesh::Point A;
    MyMesh::Point B;
    MyMesh::Point C;
    for (unsigned i=0; i<vh.size(); i++)
    {
        if (vh[i] == vhId) {
            A = _mesh->point (vh[i]);
            int k=i+1;
            if (k>=vh.size()) k=0;
            B = _mesh->point (vh[k]);
            k++;
            if (k>=vh.size()) k=0;
            C = _mesh->point (vh[k]);
            break;
        }
    }
    v1 = B-A;
    v2 = C-A;
    v1.normalize();
    v2.normalize();

    float angle = acos((v1 | v2));

    return angle;
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

float MainWindow::aire_maillage(MyMesh *_mesh)
{
    float aireTotale=0.f;
    for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
    {
        FaceHandle fh = *f_it;
        aireTotale += faceArea(_mesh, fh.idx());
    }
    return aireTotale;
}


/*-------------------------------------------------------------------------
 * Renvoi la normale du sommet d'indice @vertexID
 * (pas normalisée)
 * ----------------------------------------------------------------------*/
MyMesh::Point MainWindow::normale_sommet(MyMesh *_mesh, int vertexID)
{
    VertexHandle vh = _mesh->vertex_handle(vertexID);
    _mesh->request_face_normals();
    _mesh->request_vertex_normals();
    MyMesh::Point p = _mesh->calc_vertex_normal(vh);
    mesh.release_vertex_normals();
    return p;
}

float MainWindow::calculateCurveOnVertex(MyMesh* _mesh, int vertexID)
{
    VertexHandle vertex = VertexHandle(vertexID);
    float sum = 0.0;

    // on stocke toutes les faces adjacentes au sommet
    std::vector<int> faces;
    for(MyMesh::VertexFaceCWIter vfit = _mesh->vf_cwiter(vertex); vfit.is_valid(); vfit++){
        faces.push_back((*vfit).idx());
    }

    for(int i = 0; i < faces.size(); i++) {
        // on compare toutes les paires de face
        int face0 = faces[i];
        int face1 = faces[(i+1) % faces.size()];

        // on cherche l'arête commune entre les deux faces
        int edgeCommune;
        for(MyMesh::FaceEdgeIter feit0 = _mesh->fe_iter(FaceHandle(face0)); feit0.is_valid(); feit0++) {
            for(MyMesh::FaceEdgeIter feit1 = _mesh->fe_iter(FaceHandle(face1)); feit1.is_valid(); feit1++) {
                if((*feit0).idx() == (*feit1).idx()) {
                    edgeCommune = (*feit0).idx();
                    break;
                }
            }
        }

        // on cherche le sommet opposé sur la même arête
        int vertexOppose;
        for(MyMesh::VertexVertexIter vvit = _mesh->vv_iter(vertex); vvit.is_valid(); vvit++) {
            for(MyMesh::VertexEdgeIter veit = _mesh->ve_iter(vvit); veit.is_valid(); veit++) {
                if((*veit).idx() == edgeCommune) {
                    vertexOppose = (*vvit).idx();
                    break;
                }
            }
        }

        // on a tout trouvé, on ajoute l'angle entre les deux faces que multiplie
        // la longueur de l'arête commune
        sum += (angleFF(_mesh, face0, face1, vertexID, vertexOppose) * _mesh->calc_edge_length(EdgeHandle(edgeCommune)));
    }

    return sum;
}

void MainWindow::H_Curv(MyMesh* _mesh)
{
    // courbure moyenne
    for (MyMesh::VertexIter curVert = _mesh->vertices_begin(); curVert != _mesh->vertices_end(); curVert++) {
        float aireBar = aire_barycentrique(_mesh, (*curVert).idx());
        float H = calculateCurveOnVertex(_mesh, (*curVert).idx()) / (4 * aireBar);
        _mesh->data(*curVert).value = H;
    }
}

void MainWindow::K_Curv(MyMesh* _mesh)
{
    /* **** à compléter ! **** */

    qDebug() << "<" << __FUNCTION__ << ">";
    for (MyMesh::VertexIter curVert = _mesh->vertices_begin(); curVert != _mesh->vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        float a = 1 / aire_barycentrique(_mesh, vh.idx());
        //qDebug() << "a = " << a;
        float theta = 0.f;
        for (MyMesh::VertexFaceCWIter vf_it = _mesh->vf_cwiter(vh); vf_it.is_valid(); vf_it++)
        {
            FaceHandle fh = *vf_it;
            theta += angleEE(_mesh, vh.idx(), fh.idx());
        }
        float b = 2*M_PI - theta;
        //qDebug() << "b = " << b;
        float K = a*b;
        //qDebug() << "K = " << K;

        _mesh->data(vh).value = K;
        qDebug() << K;
    }
    qDebug() << "</" << __FUNCTION__ << ">";
}



/********************************** SIGNAUX *******************************************************/

/* **** début de la partie boutons et IHM **** */
void MainWindow::on_pushButton_H_clicked()
{
    H_Curv(&mesh);
    displayMesh(&mesh, true); // true permet de passer en mode "carte de temperatures", avec une gestion automatique de la couleur (voir exemple)
}

void MainWindow::on_pushButton_K_clicked()
{
    K_Curv(&mesh);
    displayMesh(&mesh, true); // true permet de passer en mode "carte de temperatures", avec une gestion automatique de la couleur (voir exemple)
    //displayMesh(&mesh, false); // true permet de passer en mode "carte de temperatures", avec une gestion automatique de la couleur (voir exemple)
}


void MainWindow::on_pushButton_chargement_clicked()
{
    // fenêtre de sélection des fichiers
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Mesh"), "", tr("Mesh Files (*.obj)"));

    // chargement du fichier .obj dans la variable globale "mesh"
    OpenMesh::IO::read_mesh(mesh, fileName.toUtf8().constData());

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
