
#include "utilsPcl.h"

PointCloud<PointXYZ>::Ptr UtilsPcl::loadCloud (char *fileName)
{
    PointCloud<PointXYZ>::Ptr myCloud (new PointCloud<PointXYZ>);

    std::string file = fileName;
    std::size_t pos = file.find_last_of(".");
    std::string ext = file.substr(pos);
    std::cout << ext << std::endl;

    if(ext.compare(".stl") == 0) // Si c'est un fichier au format STL
    {
        // vtk reader
        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
        reader->SetFileName(fileName);
        reader->Update();

        // convert vtk to pcl object
        io::vtkPolyDataToPointCloud(polydata, *myCloud);
    }
    else if(ext.compare(".pcd")==0) // Si c'est un fichier au format PCD
    {
        if (io::loadPCDFile<PointXYZ> (file, *myCloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            exit (1);
        }
    }
    else
    {
        std::cout << "Les format supportés sont STL et PCD" << std::endl;
        exit(1);
    }

    return myCloud;
}

bool UtilsPcl::loadCloud_from_xyzFile (const string &filename, PointCloud<PointXYZ> &cloud)
{
    ifstream fs;
    fs.open (filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }

    string line;
    std::vector<string> st;

    while (!fs.eof ())
    {
        getline (fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r ")/*, boost::token_compress_on*/);

        if (st.size () != 3)
            continue;

        cloud.push_back (PointXYZ (float (atof (st[0].c_str ())), float (atof (st[1].c_str ())), float (atof (st[2].c_str ()))));
    }
    fs.close ();

    return (true);
}

bool UtilsPcl::save_mesh (pcl::PolygonMesh& input, std::string output_file, int output_type)
{
    if (boost::filesystem::path (output_file).extension () == ".obj")
    {
        if (output_type == BINARY || output_type == BINARY_COMPRESSED)
            PCL_WARN ("OBJ file format only supports ASCII.\n");

        //TODO: Support precision
        //FIXME: Color is lost during conversion (OBJ supports color)
        PCL_INFO ("Saving file %s as ASCII.\n", output_file.c_str ());
        if (pcl::io::saveOBJFile (output_file, input) != 0)
            return (false);
    }
    else if (boost::filesystem::path (output_file).extension () == ".pcd")
    {
        if (!input.polygons.empty ())
            PCL_WARN ("PCD file format does not support meshes! Only points be saved.\n");
        pcl::PCLPointCloud2::Ptr cloud = boost::make_shared<pcl::PCLPointCloud2> (input.cloud);
        if (!UtilsPcl::save_point_cloud (cloud, output_file, output_type))
            return (false);
    }
    else  // PLY, STL and VTK
    {
        if (output_type == BINARY_COMPRESSED)
            PCL_WARN ("PLY, STL and VTK file formats only supports ASCII and binary output file types.\n");

        if (input.polygons.empty() && boost::filesystem::path (output_file).extension () == ".stl")
        {
            PCL_ERROR ("STL file format does not support point clouds! Aborting.\n");
            return (false);
        }

        PCL_INFO ("Saving file %s as %s.\n", output_file.c_str (), (output_type == ASCII) ? "ASCII" : "binary");
        if (!pcl::io::savePolygonFile (output_file, input, output_type != ASCII))
            return (false);
    }

    return (true);
}

bool UtilsPcl::save_point_cloud(const pcl::PCLPointCloud2::Ptr &input, std::string output_file, int output_type)
{
    if (boost::filesystem::path (output_file).extension () == ".pcd")
    {
        //TODO Support precision, origin, orientation
        pcl::PCDWriter w;
        if (output_type == ASCII)
        {
            PCL_INFO ("Saving file %s as ASCII.\n", output_file.c_str ());
            if (w.writeASCII (output_file, *input) != 0)
                return (false);
        }
        else if (output_type == BINARY)
        {
            PCL_INFO ("Saving file %s as binary.\n", output_file.c_str ());
            if (w.writeBinary (output_file, *input) != 0)
                return (false);
        }
        else if (output_type == BINARY_COMPRESSED)
        {
            PCL_INFO ("Saving file %s as binary compressed.\n", output_file.c_str ());
            if (w.writeBinaryCompressed (output_file, *input) != 0)
                return (false);
        }
    }
    else if (boost::filesystem::path (output_file).extension () == ".stl")
    {
        PCL_ERROR ("STL file format does not support point clouds! Aborting.\n");
        return (false);
    }
    else  // OBJ, PLY and VTK
    {
        //TODO: Support precision
        //FIXME: Color is lost during OBJ conversion (OBJ supports color)
        pcl::PolygonMesh mesh;
        mesh.cloud = *input;
        if (!UtilsPcl::save_mesh (mesh, output_file, output_type))
            return (false);
    }

    return (true);
}

void UtilsPcl::convert_file(const string &filenameInput, const string &filenameOutput)
{
    bool cloud_output (false);

    std::string parsed_output_type("ascii");

    // Convert parsed output type to output type
    int output_type (BINARY);
    if (!parsed_output_type.empty ())
    {
        if (parsed_output_type == "ascii")
            output_type = ASCII;
        else if (parsed_output_type == "binary")
            output_type = BINARY;
        else if (parsed_output_type == "binary_compressed")
            output_type = BINARY_COMPRESSED;
        else
        {
            PCL_ERROR ("Wrong output type!\n");
            //displayHelp (argc, argv);
            exit(1);
        }
    }

    // Try to load as mesh
    pcl::PolygonMesh mesh;
    if (boost::filesystem::path (filenameInput).extension () != ".pcd" &&
            pcl::io::loadPolygonFile (filenameInput, mesh) != 0)
    {
        PCL_INFO ("Loaded a mesh with %d points (total size is %d) and the following channels:\n%s\n",
                  mesh.cloud.width * mesh.cloud.height, mesh.cloud.data.size (), pcl::getFieldsList (mesh.cloud).c_str ());

        if (cloud_output)
            mesh.polygons.clear();

        if (!save_mesh (mesh, filenameOutput, output_type))
            PCL_ERROR ("Error with save_mesh\n");
            exit (1);
    }
    else if (boost::filesystem::path (filenameInput).extension () == ".stl")
    {
        PCL_ERROR ("Unable to load %s.\n", filenameInput);
        exit(1);
    }

    else
    {
        // PCD, OBJ, PLY or VTK
        if (boost::filesystem::path (filenameInput).extension () != ".pcd")
            PCL_WARN ("Could not load %s as a mesh, trying as a point cloud instead.\n", filenameInput);

        //Eigen::Vector4f origin; // TODO: Support origin/orientation
        //Eigen::Quaternionf orientation;
        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
        if (pcl::io::load (filenameInput, *cloud) < 0)
        {
            PCL_ERROR ("Unable to load %s.\n", filenameInput);
            exit(1);
        }

        PCL_INFO ("Loaded a point cloud with %d points (total size is %d) and the following channels:\n%s\n", cloud->width * cloud->height, cloud->data.size (),
                  pcl::getFieldsList (*cloud).c_str ());

        if (!save_point_cloud (cloud, filenameOutput, output_type))
        {
            PCL_ERROR ("Failed to save %s.\n", filenameOutput);
            exit(1);
        }
    }
}

PointCloud<PointXYZ> UtilsPcl::downSample_cloud (PointCloud<PointXYZ>::Ptr originalCloud)
{
    PointCloud<PointXYZ> reductCloud;
    VoxelGrid<PointXYZ> grid;
    grid.setLeafSize (1, 1, 1);
    grid.setInputCloud (originalCloud);
    grid.filter (reductCloud);

    return reductCloud;
}

void UtilsPcl::compute (PointCloud<PointXYZ> &cloud_a, PointCloud<PointXYZ> &cloud_b)
{
    typedef PointXYZ PointType;
    typedef PointCloud<PointXYZ> Cloud;
    // Estimate
    TicToc tt;
    tt.tic ();

    print_highlight (stderr, "Computing ");

    // compare A to B
    pcl::search::KdTree<PointType> tree_b;
    tree_b.setInputCloud (cloud_b.makeShared ());
    float max_dist_a = -std::numeric_limits<float>::max ();
    for (size_t i = 0; i < cloud_a.points.size (); ++i)
    {
        std::vector<int> indices (1);
        std::vector<float> sqr_distances (1);

        tree_b.nearestKSearch (cloud_a.points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
            max_dist_a = sqr_distances[0];
    }

    // compare B to A
    pcl::search::KdTree<PointType> tree_a;
    tree_a.setInputCloud (cloud_a.makeShared ());
    float max_dist_b = -std::numeric_limits<float>::max ();
    for (size_t i = 0; i < cloud_b.points.size (); ++i)
    {
        std::vector<int> indices (1);
        std::vector<float> sqr_distances (1);

        tree_a.nearestKSearch (cloud_b.points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
            max_dist_b = sqr_distances[0];
    }

    max_dist_a = std::sqrt (max_dist_a);
    max_dist_b = std::sqrt (max_dist_b);

    float dist = std::max (max_dist_a, max_dist_b);

    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
    print_info ("A->B: "); print_value ("%f", max_dist_a);
    print_info (" B->A: "); print_value ("%f", max_dist_b);
    print_info (" Hausdorff Distance: "); print_value ("%f", dist);
    print_info (" ]\n");
}

void UtilsPcl::vizu (PointCloud<PointXYZ> cloud_source, 
                     PointCloud<PointXYZ> cloud_target,
                     PointCloud<PointXYZ> cloud_icp,
                     int iterations)
{
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP");

    PointCloud<PointXYZ>::Ptr cloud_source_vizu (new PointCloud<PointXYZ>);
    copyPointCloud (cloud_source, *cloud_source_vizu);
    PointCloud<PointXYZ>::Ptr cloud_target_vizu (new PointCloud<PointXYZ>);
    copyPointCloud (cloud_target, *cloud_target_vizu);
    PointCloud<PointXYZ>::Ptr cloud_icp_vizu (new PointCloud<PointXYZ>);;
    copyPointCloud (cloud_icp, *cloud_icp_vizu);

    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is black
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h
            (cloud_target_vizu,
             0,
             255,
             0);
    viewer.addPointCloud (cloud_target_vizu, cloud_tr_color_h, "cloud_tr_v1", v1);
    viewer.addPointCloud (cloud_target_vizu, cloud_tr_color_h, "cloud_tr_v2", v2);
    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud_source_vizu, 20, 180, 20);
    viewer.addPointCloud (cloud_source_vizu, cloud_in_color_h, "cloud_in_v1", v1);
    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h (cloud_icp_vizu, 180, 20, 20);
    viewer.addPointCloud (cloud_icp_vizu, cloud_icp_color_h, "cloud_icp_v2", v2);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_in_v1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_icp_v2");

    // Adding text descriptions in each viewport
    viewer.addText ("Black: cloud target\nGreen: cloud source", 10, 15, 16, 255, 255, 255, "icp_info_1", v1);
    viewer.addText ("Black: cloud target\nRed: ICP aligned point cloud", 10, 15, 16, 255, 255, 255, "icp_info_2", v2);
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, 255, 255, 255, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (255, 255, 255, v1);
    viewer.setBackgroundColor (255, 255, 255, v2);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

