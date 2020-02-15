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
    Contour();
    Contour(unsigned _begin);
    Contour(vector<unsigned> _vertices);

    unsigned get_start();
    unsigned get_end();
    vector<unsigned> get_contour();

    SelectedPoints selected_points;

    void add_edge(unsigned numEdge);
    void add_vertex(unsigned numVertex);
    void draw_contour(MyMesh *_mesh, MyMesh::Point _sightPoint);

    void display(int profDisplay);

protected:
    int startPoint;
    int endPoint;
    vector<unsigned> edgesContour;
    vector<unsigned> verticesContour;

};

#endif // CONTOUR_H
