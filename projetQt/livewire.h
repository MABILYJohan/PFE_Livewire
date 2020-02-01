#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"

#include <list>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class LiveWire
{
public:
    LiveWire(MyMesh &_mesh);

    // Exemple
    void segmenter();

    void build();

private:
    MyMesh &mesh;

    double cost_function(int numEdgeCur, int numEdgeNeigh);
};

#endif // LIVEWIRE_H
