#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFileDialog>
#include <QMainWindow>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <vector>
#include <iostream>

#include <math.h>

namespace Ui {
class MainWindow;
}

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

struct MyTraits : public OpenMesh::DefaultTraits
{
    // use vertex normals and vertex colors
    VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
    // store the previous halfedge
    HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );
    // use face normals face colors
    FaceAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
    EdgeAttributes( OpenMesh::Attributes::Color );
    // vertex thickness
    VertexTraits{float thickness;};
    // edge thickness
    EdgeTraits{float thickness;};
};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void displayMesh(MyMesh *_mesh);
    void resetAllColorsAndThickness(MyMesh* _mesh);

    float angleEE(MyMesh* _mesh, int vertexID,  int faceID);
    float faceArea(MyMesh* _mesh, int faceID);
    float aire_barycentrique(MyMesh* _mesh, int vertID);
    double K_Curv(MyMesh* _mesh, int vertID);

private slots:

    void on_pushButton_chargement_clicked();

    void on_pushButton_clicked();

private:

    bool modevoisinage;

    MyMesh mesh;

    int vertexSelection;
    int edgeSelection;
    int faceSelection;

    std::vector<bool> tabvertex;

    std::vector<double> tabcurv;

    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
