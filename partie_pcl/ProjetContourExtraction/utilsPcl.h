#ifndef UTILSPCL_H
#define UTILSPCL_H

/*!
 * \file utilsPcl.h
 * \brief contains utilities functions that works with PCL library
 * \author Johan MABILY
 * \author Erwan LERIA
 * \author Pierre MATTIOLI
 * \version 1.0
 */

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

/*! \class UtilsPcl
   * \brief class which contains abstract functions that works with pcl
   */
class UtilsPcl
{
public:
    /**
     * \brief Builder
     */
    UtilsPcl();
    
    /**
     * \brief Loader
     *
     * load .pcd or .stl files and convert into points cloud
     *
     * \param fileName : path of the file
     * \return a pointer on the points cloud of type PointCloud<PointXYZ>::Ptr
     */
    static PointCloud<PointXYZ>::Ptr loadCloud (char *fileName);

    /**
     * \brief Loader xyz file
     *
     * load .xyz file and extract the points cloud
     *
     * \param fileName : path of the file
     * \param cloud : The variable wich will contains the cloud
     *
     * \return the cloud of type PointCloud<PointXYZ> &
     */
    static bool loadCloud_from_xyzFile (const string &filename, PointCloud<PointXYZ> &cloud);

    static PointCloud<PointXYZ> downSample_cloud (PointCloud<PointXYZ>::Ptr originalCloud);
    static void compute (PointCloud<PointXYZ> &cloud_a, PointCloud<PointXYZ> &cloud_b);
    static void vizu (PointCloud<PointXYZ> cloud_source,
                      PointCloud<PointXYZ> cloud_target,
                      PointCloud<PointXYZ> cloud_icp,
                      int iterations);
};



#endif

