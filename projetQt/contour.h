#ifndef CONTOUR_H
#define CONTOUR_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

#include "utilsMesh.h"
#include "selectedpoints.h"
#include "livewire.h"

#include <vector>
#include <qdebug.h>

using namespace std;

class Contour
{
public:
    Contour(MyMesh &_mesh);
    Contour(MyMesh &_mesh, unsigned _begin);
    Contour(MyMesh &_mesh, vector<unsigned> _vertices);
    Contour(MyMesh &_mesh, char *path);

    unsigned get_start();
    unsigned get_end();
    vector<unsigned> get_contour();

    void add_edge(unsigned numEdge);
    void add_vertex(unsigned numVertex);
    void draw_contour(MyMesh *_mesh, MyMesh::Point _sightPoint);

    //    MyMesh load_points_with_mesh(char *path);

    double half_thickness = 2; //Radius of pre-supposed brush radius which "draws" the stroke

    void display(int profDisplay, bool flagColor=false);

protected:

    MyMesh &mesh;

    int startPoint;
    int endPoint;
    vector<unsigned> edgesContour;
    vector<unsigned> verticesContour;

    int search_max_dist_vertex_from_vertex(vector<int> tmp, int id);
};

#endif // CONTOUR_H
