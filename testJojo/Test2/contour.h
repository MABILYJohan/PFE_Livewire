#ifndef CONTOUR_H
#define CONTOUR_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

#include "utilsMesh.h"

#include <vector>

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

    void add_edge(unsigned numEdge);

protected:

    unsigned startPoint;
    unsigned endPoint;
    vector<unsigned> contour;
};

#endif // CONTOUR_H
