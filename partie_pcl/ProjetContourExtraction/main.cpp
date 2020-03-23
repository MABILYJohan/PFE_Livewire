/*!
 * \file main.cpp
 * \brief module principal pour lancer le programme
 * \author Johan MABILY
 * \author Erwan LERIA
 * \author Pierre MATTIOLI
 * \version 1.0
 */

#include <iostream>
#include <fstream>

#include "extractContour.h"
#include "utilsPcl.h"

/**
* \brief Display help
*
* Display how to use the program
*/
void printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.xyz[or .txt] output.pcd\n", argv[0]);
}

/**
* \brief Main function of the program
*
* Read .xyz file and extract borders of the points cloud.
* \return contour.xyz, contour.obj
*/
int main(int argc, char** argv)
{

    // --------------
    // -----INIT-----
    // --------------
    print_info ("Convert a simple XYZ file to PCD format. For more information, use: %s -h\n", argv[0]);
    if (argc < 3)
    {
        printHelp (argc, argv);
        return (-1);
    }
    // Parse the command line arguments for .pcd and .ply files
    std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    std::vector<int> xyz_file_indices = parse_file_extension_argument (argc, argv, ".xyz");
    if (xyz_file_indices.empty()) {
        xyz_file_indices = parse_file_extension_argument (argc, argv, ".txt");
    }
    if (pcd_file_indices.size () != 1 || xyz_file_indices.size () != 1)
    {
        print_error ("Need one input XYZ (or txt) file and one output PCD file.\n");
        return (-1);
    }
    // Load the first file
    PointCloud<PointXYZ> cloud;
    if (!UtilsPcl::loadCloud_from_xyzFile (argv[xyz_file_indices[0]], cloud))
        return (-1);


    // ---------------------------------
    // -----Convert to pcd and save-----
    // ---------------------------------
    PCDWriter w;
    w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);

    // -------------------------------------------
    // -----Convert original to .obj -------------
    // -------------------------------------------
    PointCloud<PointXYZ> cloud3;
    // Contour to pcd
    if (!UtilsPcl::loadCloud_from_xyzFile (argv[xyz_file_indices[0]], cloud3))
        return (-1);
    PCDWriter w3;
    w3.writeBinaryCompressed ("meshBase.pcd", cloud3);
    UtilsPcl::convert_file("meshBase.pcd", "meshBase.obj");


    // -------------------------------------------------
    // -----Extract borders and save into .xyz file-----
    // -------------------------------------------------
    ExtractContour myExtractor(0.5, true, true);
    myExtractor.extract(argv[pcd_file_indices[0]]);


    // ----------------------------------
    // -----Convert contour to .obj -----
    // ----------------------------------
    PointCloud<PointXYZ> cloud2;
    // Contour to pcd
    if (!UtilsPcl::loadCloud_from_xyzFile ("contour.xyz", cloud2))
        return (-1);
    PCDWriter w2;
    w2.writeBinaryCompressed ("contour.pcd", cloud2);
    UtilsPcl::convert_file("contour.pcd", "contour.obj");


    // ----------------------------------
    // -----Remove unutils files --------
    // ----------------------------------
    remove(argv[pcd_file_indices[0]]);
    remove("contour.pcd");
    remove("meshBase.pcd");

    return 0;
}
