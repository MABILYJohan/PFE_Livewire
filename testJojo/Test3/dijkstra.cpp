#include "dijkstra.h"

#include <QDebug>

Dijkstra::Dijkstra()
{

}

vector<int> Dijkstra::get_paths()   {   return  paths;  }

bool Dijkstra::tous_sommets_extraits(QVector<bool> tab)
{
    for(int i=0; i<tab.size(); i++){
        if(tab[i]){
            return false;
        }
    }
    return true;
}

int Dijkstra::extraire_min(QVector<bool> listeSommets)
{
    int poid_min = INT32_MAX, sommet_min;
    for(int i=0; i<listeSommets.size(); i++){
        if(listeSommets[i] && (tabWeights[i] <= poid_min)){
            poid_min = tabWeights[i];
            sommet_min = i;
        }
    }
    return sommet_min;
}

void Dijkstra::dijkstra(MyMesh* _mesh, int sommetDepart)
{
    int nbVertex = _mesh->n_vertices();
    QVector<bool> vertexList;
    paths.clear();
    tabWeights.clear();

    // INIT
    for(int v=0; v<nbVertex; v++){
        //On initialise nos valeurs pour utiliser dijkstra
        vertexList.append(true);
        tabWeights.push_back(INT32_MAX);
        paths.push_back(-1);
    }
    tabWeights[sommetDepart] = 0;

    while(!tous_sommets_extraits(vertexList))
    {
        int vertex = extraire_min(vertexList);
        //qDebug() << "vertex = " << vertex;
        vertexList[vertex] = false;
        VertexHandle vhVertex = _mesh->vertex_handle(vertex);

        for (MyMesh::VertexVertexIter vv_it=_mesh->vv_iter(vhVertex); vv_it.is_valid(); vv_it++)
        {
            int poids = tabWeights[vertex] + 1;
            if(tabWeights[vv_it->idx()] > poids){
                //MAJ du poids
                tabWeights[vv_it->idx()] = poids;
                paths[vv_it->idx()] = vertex;
            }
        }
    }
//    paths.clear();
//    for (auto p : pred) {
//        paths.push_back(p);
//    }
}
