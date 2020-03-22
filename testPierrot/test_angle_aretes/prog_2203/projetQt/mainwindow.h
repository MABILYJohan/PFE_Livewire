#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFileDialog>
#include <QMainWindow>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utilsMesh.h"
#include "contour.h"
#include "livewire.h"


namespace Ui {
class MainWindow;
}

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
    int testChoice=1;
	MyMesh::Point get_pt_de_vue();

    void displayMesh(MyMesh *_mesh, bool isTemperatureMap = false, float mapRange = -1);
    void resetAllColorsAndThickness(MyMesh* _mesh);

private slots:
    void on_pushButton_chargement_clicked();

    void on_pushButton_generer_clicked();

    void on_pushButton_livewire_clicked();
    void on_pushButton_vizuContour_clicked();

    void on_spinBox_valueChanged(int value);

private:

    MyMesh mesh;

    Ui::MainWindow *ui;

    vector<unsigned> get_verticesID(int _testChoice);
    vector<unsigned> test_contour(vector<MyMesh::Point> points, int precision);

    void make_livewire();
    void vizuContour(int displayDist=0);
};

#endif // MAINWINDOW_H
