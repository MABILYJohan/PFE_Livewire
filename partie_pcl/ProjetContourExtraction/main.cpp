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
* Read .xyz file, and convert into .pcd file, and extract borders into an other .xyz file.
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

    // ----------------------------------
    // -----Extract borders and save-----
    // ----------------------------------
    ExtractContour myExtractor(0.5, true, false);
    myExtractor.extract(argv[pcd_file_indices[0]]);

    return 0;
}
