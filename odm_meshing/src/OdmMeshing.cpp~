#include "OdmMeshing.hpp"


OdmMeshing::OdmMeshing() : log_(false)
{
    meshCreator_ = pcl::Poisson<pcl::PointNormal>::Ptr(new pcl::Poisson<pcl::PointNormal>());
    points_  = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    mesh_   = pcl::PolygonMeshPtr(new pcl::PolygonMesh);
    decimatedMesh_   = pcl::PolygonMeshPtr(new pcl::PolygonMesh);

    // Set default values
    outputFile_ = "";
    logFilePath_ = "";

    maxVertexCount_ = 0;
    treeDepth_ = 0;

    solverDivide_ = 9.0;
    samplesPerNode_ = 1.0;
    decimationFactor_ = 0.0;

    logFilePath_ = "odm_meshing_log.txt";
    log_ << logFilePath_ << "\n";
}

OdmMeshing::~OdmMeshing()
{

}

int OdmMeshing::run(int argc, char **argv)
{
    // If no arguments were passed, print help and return early.
    if (argc <= 1)
    {
        printHelp();
        return EXIT_SUCCESS;
    }

    try
    {
        parseArguments(argc, argv);

        loadPoints();

        createMesh();

        decimateMesh();

        writePlyFile();

    }
    catch (const OdmMeshingException& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << e.what() << "\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in OdmMeshing:\n";
        log_ << e.what() << "\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (...)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Unknwon error in OdmMeshing:\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }

    log_.printToFile(logFilePath_);
    return EXIT_SUCCESS;
}


void OdmMeshing::parseArguments(int argc, char **argv)
{

    for(int argIndex = 1; argIndex < argc; ++argIndex)
    {
        // The argument to be parsed.
        std::string argument = std::string(argv[argIndex]);

        if(argument == "-help")
        {
            printHelp();
        }
        else if(argument == "-verbose")
        {
            log_.setIsPrintingInCout(true);
        }
        else if(argument == "-maxVertexCount" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> maxVertexCount_;
            if (ss.bad())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            }
            log_ << "Vertex count was manually set to: " << maxVertexCount_ << "\n";
        }
        else if(argument == "-octreeDepth" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> treeDepth_;
            if (ss.bad())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            }
            log_ << "Octree depth was manually set to: " << treeDepth_ << "\n";
        }
        else if(argument == "-solverDivide" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> solverDivide_;
            if (ss.bad())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            }
            log_ << "Numerical solver divisions was manually set to: " << treeDepth_ << "\n";
        }
        else if(argument == "-samplesPerNode" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> samplesPerNode_;
            if (ss.bad())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            }
            log_ << "The number of samples per octree node was manually set to: " << samplesPerNode_ << "\n";
        }
        else if(argument == "-inputFile" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputFile_ = std::string(argv[argIndex]);
            std::ifstream testFile(inputFile_.c_str(), std::ios::binary);
            if (!testFile.is_open())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value. (file not accessible)");
            }
            testFile.close();
            log_ << "Reading point cloud at: " << inputFile_ << "\n";
        }
        else if(argument == "-outputFile" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputFile_ = std::string(argv[argIndex]);
            std::ofstream testFile(outputFile_.c_str());
            if (!testFile.is_open())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value.");
            }
            testFile.close();
            log_ << "Writing output to: " << outputFile_ << "\n";
        }
        else if(argument == "-logFile" && argIndex < argc)
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            logFilePath_ = std::string(argv[argIndex]);
            std::ofstream testFile(outputFile_.c_str());
            if (!testFile.is_open())
            {
                throw OdmMeshingException("Argument '" + argument + "' has a bad value.");
            }
            testFile.close();
            log_ << "Writing log information to: " << logFilePath_ << "\n";
        }
        else
        {
            printHelp();
            throw OdmMeshingException("Unrecognised argument '" + argument + "'");
        }
    }
}

void OdmMeshing::loadPoints()
{

    if(pcl::io::loadPLYFile<pcl::PointNormal> (inputFile_.c_str(), *points_.get()) == -1) {
        throw OdmMeshingException("Error when reading points and normals from:\n" + inputFile_ + "\n");
    }
    else
    {
        log_ << "Successfully loaded " << points_->size() << " points with corresponding normals from file.\n";
    }
}

void OdmMeshing::printHelp()
{
    bool printInCoutPop = log_.isPrintingInCout();
    log_.setIsPrintingInCout(true);

    log_ << "OpenDroneMapMeshing.exe\n\n";

    log_ << "Purpose:" << "\n";
    log_ << "Create a mesh from an oriented point cloud (points with normals) using the Poisson surface reconstruction method." << "\n";

    log_ << "Usage:" << "\n";
    log_ << "The program requires a path to an input PLY point cloud file, all other input parameters are optional." << "\n\n";

    log_ << "The following flags are available\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";

    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: " << "\n";
    log_ << "\"-inputFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input ascii ply file that must contain a point cloud with normals.\n\n";

    log_ << "\"-outputFile <path>\" (optional, default: odm_mesh.ply)" << "\n";
    log_ << "\"Target file in which the mesh is saved.\n\n";

    log_ << "\"-logFile <path>\" (optional, default: odm_meshing_log.txt)" << "\n";
    log_ << "\"Target file in which the mesh is saved.\n\n";

    log_ << "\"-maxVertexCount <integer>\" (optional, default: 100,000)" << "\n";
    log_ << "Desired final vertex count (after decimation), set to 0 to disable decimation.\n\n";

    log_ << "\"-treeDepth <integer>\" (optional, default: 0 (automatic))" << "\n";
    log_ << "Controls octree depth used for poisson reconstruction. Recommended values (9-11).\n"
         << "Increasing the value on this parameter will raise initial vertex count."
         << "If omitted or zero, the depth is calculated automatically from the input point count.\n\n";

    log_ << "\"-samplesPerNode <float>\" (optional, default: 1)" << "\n";
    log_ << "Average number of samples (points) per octree node. Increasing this value might help if data is very noisy.\n\n";

    log_ << "\"-solverDivide <integer>\" (optional, default: 9)" << "\n";
    log_ << "Ocree depth at which the Laplacian equation is solved in the surface reconstruction step.\n";
    log_ << "Increasing this value increases computation times slightly but helps reduce memory usage.\n\n";

    log_.setIsPrintingInCout(printInCoutPop);
}

void OdmMeshing::createMesh()
{

    // Attempt to calculate the depth of the tree if unspecified
    if (treeDepth_ == 0)
    {
        treeDepth_ = calcTreeDepth(points_->size());
    }

    log_ << "Octree depth used for reconstruction is: " << treeDepth_ << "\n";
    log_ << "Estimated initial vertex count: " << pow(4, treeDepth_) << "\n\n";

    meshCreator_->setDepth(treeDepth_);
    meshCreator_->setSamplesPerNode(samplesPerNode_);
    meshCreator_->setInputCloud(points_);

    // Guarantee manifold mesh.
    meshCreator_->setManifold(true);

    // Begin reconstruction
    meshCreator_->reconstruct(*mesh_.get());

    log_ << "Reconstruction complete:\n";
    log_ << "Vertex count: " << mesh_->cloud.width << "\n";
    log_ << "Triangle count: " << mesh_->polygons.size() << "\n\n";

}

void OdmMeshing::decimateMesh()
{
    if (maxVertexCount_ <= 0)
    {
        log_ << "Vertex count not specified, decimation cancelled.\n";
        return;
    }

    if (maxVertexCount_ > mesh_->cloud.height*mesh_->cloud.width)
    {
        log_ << "Vertex count in mesh lower than initially generated mesh, unable to decimate.\n";
        return;
    }
    else
    {
        decimatedMesh_ = pcl::PolygonMeshPtr(new pcl::PolygonMesh);

        double reductionFactor = 1.0 - double(maxVertexCount_)/double(mesh_->cloud.height*mesh_->cloud.width);

        log_ << "Decimating mesh, removing " << reductionFactor*100 << " percent of vertices.\n";

        pcl::MeshQuadricDecimationVTK decimator;
        decimator.setInputMesh(mesh_);
        decimator.setTargetReductionFactor(reductionFactor);
        decimator.process(*decimatedMesh_.get());

        log_ << "Decimation complete.\n";
        log_ << "Decimated vertex count: " << decimatedMesh_->cloud.width << "\n";
        log_ << "Decimated triangle count: " << decimatedMesh_->polygons.size() << "\n\n";

        mesh_ = decimatedMesh_;
    }
}

int OdmMeshing::calcTreeDepth(size_t nPoints)
{
    // Assume points are located (roughly) in a plane.
    double squareSide = std::sqrt(double(nPoints));

    // Calculate octree depth such that if points were equally distributed in
    // a quadratic plane, there would be at least 1 point per octree node.
    int depth = 0;
    while(std::pow<double>(2,depth) < squareSide/2)
    {
        depth++;
    }
    return depth;
}

void OdmMeshing::writePlyFile()
{
    log_ << "Saving mesh to file.\n";
    if (pcl::io::savePLYFile(outputFile_.c_str(), *mesh_.get())  == -1) {
        throw OdmMeshingException("Error when saving mesh to file:\n" + outputFile_ + "\n");
    }
    else
    {
        log_ << "Successfully wrote mesh to:\n"
             << outputFile_ << "\n";
    }
}
