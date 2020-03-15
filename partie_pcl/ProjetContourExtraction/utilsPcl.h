#ifndef UTILSPCL_H
#define UTILSPCL_H


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

class UtilsPcl
{
	public:
    UtilsPcl();
    
    static PointCloud<PointXYZ>::Ptr loadCloud (char *fileName);

    static bool loadCloud_from_xyzFile (const string &filename, PointCloud<PointXYZ> &cloud);

    static PointCloud<PointXYZ> downSample_cloud (PointCloud<PointXYZ>::Ptr originalCloud);
    static void compute (PointCloud<PointXYZ> &cloud_a, PointCloud<PointXYZ> &cloud_b);
    static void vizu (PointCloud<PointXYZ> cloud_source, 
			PointCloud<PointXYZ> cloud_target,
			PointCloud<PointXYZ> cloud_icp,
			int iterations);
};



#endif

