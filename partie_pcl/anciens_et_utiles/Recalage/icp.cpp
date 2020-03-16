#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>

#include <string>

#include <pcl/io/ply_io.h>

#include "utilsPcl.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef PointXYZ PointT;
typedef PointCloud<PointT> PointCloudT;


int main (int argc, char** argv)
{
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    cout << " loading cloud_in " << endl;
    PointCloudT::Ptr cloud_in = UtilsPcl::loadCloud(argv[1]);
    cout << " loading cloud_out " << std::endl;
    PointCloudT::Ptr cloud_out = UtilsPcl::loadCloud(argv[2]);

    //    // Fill in the CloudIn data
    //    cloud_in->width    = 5;
    //    cloud_in->height   = 1;
    //    cloud_in->is_dense = false;
    //    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    //    for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
    //    {
    //        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    //    }
    //    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
    //              << std::endl;
    //    for (std::size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
    //                                                                            cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
    //                                                                            cloud_in->points[i].z << std::endl;

    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
              << std::endl;
    for (std::size_t i = 0; i < cloud_out->points.size (); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
                     cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    print_highlight ("Visualisation \n");
    UtilsPcl::vizu (*cloud_in, *cloud_out, Final, 1);

    return (0);
}
