#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "utilsMesh.h"

#include <QVector>
#include <vector>

using namespace std;

class Dijkstra
{
public:
    Dijkstra();

    vector<int> get_paths();

    void dijkstra(MyMesh* _mesh, int sommetDepart);

private:
    vector<int> paths;
    vector<int> tabWeights;

    bool tous_sommets_extraits(QVector<bool> tab);
    int extraire_min(QVector<bool> listeSommets);
};

#endif // DIJKSTRA_H
