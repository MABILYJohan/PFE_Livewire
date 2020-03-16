/* \author Bastian Steder */
/* \author Johan MABILY */

#ifndef EXTRACTCONTOUR_H
#define EXTRACTCONTOUR_H

/*!
 * \file extractContour.h
 * \brief contains class of border extraction.
 * \author Johan MABILY
 * \author Erwan LERIA
 * \author Pierre MATTIOLI
 * \version 1.0
 */

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

/*! \class ExtractContour
   * \brief class which extract borders of a .pcd file
   */
class ExtractContour
{
public:

    /**
     * \brief Builder
     *
     * \param _angular_resolution : angular resolution in degrees
     * \param _setUnseenToMaxRange : Treat all unseen points to max range
     * \param _b_vizu : display or not the viewer
     *
     * \return a pointer on the points cloud of type PointCloud<PointXYZ>::Ptr
     */
    ExtractContour (float _angular_resolution=0.5, bool _setUnseenToMaxRange=true,
                    bool _b_vizu=false);

    bool b_vizu = false;

    //    void read_pcd_file(int argc, char** argv);

    /**
     * \brief extract borders
     *
     * extract borders of the points cloud, and put it into a .xyz file
     *
     * \param filename : the path of the file
     */
    void extract(const string &filename);

private:
    float angular_resolution = 0.5f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = true;
};



#endif

