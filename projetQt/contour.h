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
    Contour(vector<unsigned> _edges);

    unsigned get_start();
    unsigned get_end();
    vector<unsigned> get_contour();

    SelectedPoints selected_points;

    void add_edge(unsigned numEdge);
    void draw(MyMesh *_mesh);

protected:
    int startPoint;
    int endPoint;
    vector<unsigned> contour;
};

#endif // CONTOUR_H
