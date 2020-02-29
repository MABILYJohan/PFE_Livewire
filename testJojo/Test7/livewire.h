#ifndef LIVEWIRE_H
#define LIVEWIRE_H

/*!
 * \file LiveWire.h
 * \brief Module de calcul de LiveWire 3D sur un maillage
 * \author Johan MABILY
 * \author Erwan LERIA
 * \author Pierre MATTIOLI
 * \version alpha
 */

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"
#include "dijkstra.h"

#include <math.h>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

enum enum_criteres {LENGTH, DIEDRAL, CURVATURE, NORMAL_OR, VISIBILITY, STROKE_DIST};

/*! \class LiveWire
   * \brief classe representant le LiveWire
   *
   *  La classe calcule l'algorithme du livewire3D sur un maillage
   */
class LiveWire
{

public:

    /**
     * \brief Constructeur
     *
     * Constructeur de la classe LiveWire
     *
     * \param _mesh : le maillage auquel on va appliquer l'algorithme
     * \param _vertexSeed : Le sommet de départ de l'algo
     * \param _sightPoint : Le point de vue utilisateur
     */
    LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint);

    /**
     * \brief Afficheur de critères
     *
     * Affiche les critères qui vont être utilisés
     *
     * \param profDisplay : simple variable pour régler la profondeur d'affichage
     */
    void display_criterions(int profDisplay = 0);

    /**
     * \brief Met à jour le sommet de départ
     *
     * Met à jour le sommet de départ @_vertexSeed et met à jour Dijkstra
     * si critère de visibilité puis refait les chemins avec critères.
     * @vertexNext pour le critère de visibilité avec dijkstra.
     *
     * \param vertexNext : vertex suivant pour le critère de visibilité utilisant Dijkstra
     * \param close : boolean pour savoir si le chemin qui va être calculé fermera le contour
     */
    void update_vertexSeed(int _vertexSeed, int vertexNext, bool close=false);

    /**
     * \brief Dessine le chemin calculé par le LiveWire
     *
     * Dessine le chemin dans @mesh entre l'arête @edgeSeed et une arête @edge2
     *
     * \param vertex2 : sommet d'arrivé du sommet @vertexSeed
     */
    void draw(unsigned vertex2);

    /**
     * \brief Construit les chemins pour le livewire
     *
     * Construit les chemins pour le livewire (dans @paths).
     * @vertexNext pour le critère de visibilité avec dijkstra.
     * Pensez au cas où à bien mettre à jour @vertexSeed avant d'utiliser la focntion.
     *
     * \param vertexNext : vertex suivant pour le critère de visibilité utilisant Dijkstra
     * \param close : boolean pour savoir si le chemin qui va être calculé fermera le contour
    */
    void build_paths(int vertexNext, bool close=false);

    /**
     * \brief Accesseur chemins calculés
     *
     * \return tableau des prédecesseurs de chaque arêtes
     */
    vector<int> get_paths();

private:
    MyMesh &mesh;           /*!< Maillage */
    vector<int> paths;      /*!< Tableau des prédecesseurs */
    Dijkstra myDijkstra;    /*!< Instance de Dijkstra */
    int vertexSeed;         /*!< Sommet de départ */
    int edgeSeed;           /*!< Arête de départ */
    double minCurv;
    double maxCurv;
    MyMesh::Point sightPoint;   /*!< Point de vue utilisateur */

    int nbMaxCrit = 6;      /*!< Nombre maximum de critères pouvant être utilisés */
    vector<int> criteres;   /*!< Tableau d'enums */
    vector<vector<double>> tabCosts; /*!< Tableau de tableaux de coûts précalculés (en fct de critères) */
    double rad_thickness = 1.2; //Radius of pre-supposed brush radius which "draws" the stroke


    /**
     * \brief Initialisateur des différents critères
     *
     * A défaut de le faire dans l'interface, c'est ici qu'on choisit
     * les critères souhaités, et qu'on initie les tableaux de préchargement.
     */
    void init_criterions();

    /**
     * \brief Critère de longueur d'arête
     *
     * Calcule la longueur d'une arête
     *
     * \param eh : Le EdgeHandle de l'arête à calculer
     *
     * \return : la longueur de @eh
     */
    double criterion_length(EdgeHandle eh);

    /**
     * \brief Critère d'angle dièdre d'une arête
     *
     * Calcule l'angle dièdre d'une arête
     *
     * \param eh : Le EdgeHandle de l'arête à calculer
     *
     * \return : l'angle dièdre de @eh
     */
    double criterion_diedral_angle(EdgeHandle eh);

    /**
     * \brief Critère d'orientation de normale d'une arête
     *
     * Calcule l'angle entre le point de vue utilisateur et les normales
     * des faces adjacentes à l'arête.
     *
     * \param eh : Le EdgeHandle de l'arête à calculer
     * \param _sightPoint : Le point de vue utilisateur
     *
     * \return : l'angle calculé
     */
    double criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint);

    void init_min_max_curvature(MyMesh mesh);
    double criterion_curvature(EdgeHandle eh);

    /**
     * \brief Critère de visibilité d'une arête
     *
     * Donne un coût à une arête en fonction de si elle se trouve sur le
     * contour dessiné ou pas.
     * Au plus l'arête est loin du contour, au plus le coût est bas.
     * S'utilise avec Dijkstra.
     *
     * \param eh : Le EdgeHandle de l'arête à calculer
     *
     * \return : le coût
     */
    double criterion_visibility(EdgeHandle eh);


    double criterion_stroke_distance(EdgeHandle eh);

    //fonctions pour la courbure gaussienne

    /**
     * \brief Calcul d'angle entre 2 arêtes d'une face triangulaire
     *
     * Calcule l'angle entre 2 arêtes spécifiés appartenant à la même face triangulaire
     *
     *
     * \param _mesh : le maillage
     * \param vertexID : l'id du sommet entre les 2 arêtes
     * \param faceID : l'id de la face
     *
     * \return : l'angle entre les 2 arêtes souhaitées
     */
    float angleEE(MyMesh* _mesh, int vertexID,  int faceID);

    /**
     * \brief Calcul de l'aire d'une face
     *
     * \param _mesh : le maillage
     * \param faceID : l'id de la face
     *
     * \return : l'aire de la face
     */
    float faceArea(MyMesh* _mesh, int faceID);

    /**
     * \brief Calcul de l'aire barycentrique
     *
     * Calcule l'aire barycentrique autour d'un sommet
     *
     * \param _mesh : le maillage
     * \param vertID : l'id du sommet
     *
     * \return : l'aire barycentrique
     */
    float aire_barycentrique(MyMesh* _mesh, int vertID);

    /**
     * \brief Calcul de la gaussienne en un point/sommet
     *
     *
     * \param _mesh : le maillage
     * \param vertID : l'id du sommet
     *
     * \return : la valeur de la gaussienne à ce sommet
     */
    double K_Curv(MyMesh* _mesh, int vertID);

    /**
     * \brief Fonction de calcul de coût en une arête
     *
     * Calcule à l'aide des critères choisis dans l'instance, le coût attribué
     * à cette arête (en fonction du chemin courant)
     *
     * \param numEdgeNeigh : l'id d'une arête
     * \param close : boolean pour savoir si le chemin qui va être calculé fermera le contour
     *
     * \return : le coût calculé pour cette arête
     */
    double cost_function(int numEdgeNeigh, bool close=false);
};

#endif // LIVEWIRE_H
