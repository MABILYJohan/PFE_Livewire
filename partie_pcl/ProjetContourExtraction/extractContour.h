/* \author Bastian Steder */
/* \author Johan MABILY */

#ifndef EXTRACTCONTOUR_H
#define EXTRACTCONTOUR_H

#include <iostream>
#include <fstream>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef pcl::PointXYZ PointType;

class ExtractContour
{
public:
    ExtractContour (float _angular_resolution=0.5, bool _setUnseenToMaxRange=true,
                    bool _b_vizu=false);

    bool b_vizu = false;

    void read_pcd_file(int argc, char** argv);

    void extract(const string &filename);

private:
    float angular_resolution = 0.5f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = true;

    //    pcl::PointCloud<PointType>& truc;
    //    pcl::PointCloud<PointType>::Ptr point_cloud_ptr;
    //    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    //    Eigen::Affine3f scene_sensor_pose;

    //    pcl::RangeImage::Ptr range_image_ptr;

    //    pcl::visualization::PCLVisualizer viewer;

    //    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    //    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr;
    //    pcl::PointCloud<pcl::PointWithRange>::Ptr veil_points_ptr;
    //    pcl::PointCloud<pcl::PointWithRange>::Ptr shadow_points_ptr;

    void create_range_image();
    void set_viewer_3D();
    void extract_borders();
};



#endif

