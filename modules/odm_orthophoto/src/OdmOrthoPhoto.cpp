// C++
#include <math.h>
#include <sstream>
#include <fstream>
#include <Eigen/StdVector>

// This
#include "OdmOrthoPhoto.hpp"

std::ostream & operator<< (std::ostream &os, const WorldPoint &worldPoint)
{
    return os << worldPoint.eastInteger_ + worldPoint.eastFractional_ << " " << worldPoint.northInteger_ + worldPoint.northFractional_;
}

std::istream & operator>> (std::istream &is,  WorldPoint &worldPoint)
{
    is >> worldPoint.eastInteger_;
    // Check if east coordinate is given as rational.
    if('.' == is.peek())
    {
        is >> worldPoint.eastFractional_;
    }
    else
    {
        worldPoint.eastFractional_ = 0.0f;
    }
    
    is >> worldPoint.northInteger_;
    // Check if north coordinate is given as rational.
    if('.' == is.peek())
    {
        is >> worldPoint.northFractional_;
    }
    else
    {
        worldPoint.northFractional_ = 0.0f;
    }

    return is;
}

OdmOrthoPhoto::OdmOrthoPhoto()
    :log_(false)
{
    inputFile_ = "";
    inputGeoRefFile_ = "";
    inputTransformFile_ = "";
    outputFile_ = "ortho.jpg";
    logFile_    = "log.txt";
    outputCornerFile_ = "";

    transformOverride_ = false;

    resolution_ = 0.0f;

    boundaryDefined_ = false;
    boundaryPoint1_[0] = 0.0f; boundaryPoint1_[1] = 0.0f;
    boundaryPoint2_[0] = 0.0f; boundaryPoint2_[1] = 0.0f;
    boundaryPoint3_[0] = 0.0f; boundaryPoint3_[1] = 0.0f;
    boundaryPoint4_[0] = 0.0f; boundaryPoint4_[1] = 0.0f;
}

OdmOrthoPhoto::~OdmOrthoPhoto()
{
}

int OdmOrthoPhoto::run(int argc, char *argv[])
{
    try
    {
        parseArguments(argc, argv);
        createOrthoPhoto();
    }
    catch (const OdmOrthoPhotoException& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in OdmOrthoPhoto:\n";
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (...)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Unknown error, terminating:\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    
    log_.print(logFile_);
    
    return EXIT_SUCCESS;
}

void OdmOrthoPhoto::parseArguments(int argc, char *argv[])
{
    logFile_ = std::string(argv[0]) + "_log.txt";
    log_ << logFile_ << "\n\n";
    
    // If no arguments were passed, print help.
    if (argc == 1)
    {
        printHelp();
    }
    
    log_ << "Arguments given\n";
    for(int argIndex = 1; argIndex < argc; ++argIndex)
    {
        log_ << argv[argIndex] << '\n';
    }
    
    log_ << '\n';
    for(int argIndex = 1; argIndex < argc; ++argIndex)
    {
        // The argument to be parsed.
        std::string argument = std::string(argv[argIndex]);
        
        if(argument == "-help")
        {
            printHelp();
        }
        else if(argument == "-resolution")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> resolution_;
            log_ << "Resolution count was set to: " << resolution_ << "pixels/meter\n";
        }
        else if(argument == "-boundary")
        {
            if(argIndex+8 >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 8 more input following it, but no more inputs were provided.");
            }

            std::stringstream ss;
            ss << argv[argIndex+1] << " " << argv[argIndex+2] << " " << argv[argIndex+3] << " " << argv[argIndex+4] << " " << argv[argIndex+5] << " " << argv[argIndex+6] << " " << argv[argIndex+7] << " " << argv[argIndex+8];
            ss >> worldPoint1_ >> worldPoint2_ >> worldPoint3_ >> worldPoint4_;
            boundaryDefined_ = true;

            argIndex += 8;

            log_ << "Boundary point 1 was set to: " << worldPoint1_ << '\n';
            log_ << "Boundary point 2 was set to: " << worldPoint2_ << '\n';
            log_ << "Boundary point 3 was set to: " << worldPoint3_ << '\n';
            log_ << "Boundary point 4 was set to: " << worldPoint4_ << '\n';
        }
        else if(argument == "-boundaryMinMax")
        {
            if(argIndex+4 >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 4 more input following it, but no more inputs were provided.");
            }

            std::stringstream ss;
            ss << argv[argIndex+1] << " " << argv[argIndex+2] << " " << argv[argIndex+3] << " " << argv[argIndex+4];
            ss >> worldPoint1_ >> worldPoint3_;
            boundaryDefined_ = true;

            // Set the other world points as the other two corners.
            worldPoint2_.eastFractional_  = worldPoint1_.eastFractional_;
            worldPoint2_.eastInteger_     = worldPoint1_.eastInteger_;
            worldPoint2_.northFractional_ = worldPoint3_.northFractional_;
            worldPoint2_.northInteger_    = worldPoint3_.northInteger_;
            
            worldPoint4_.eastFractional_  = worldPoint3_.eastFractional_;
            worldPoint4_.eastInteger_     = worldPoint3_.eastInteger_;
            worldPoint4_.northFractional_ = worldPoint1_.northFractional_;
            worldPoint4_.northInteger_    = worldPoint1_.northInteger_;

            argIndex += 4;

            log_ << "Boundary point 1 was set to: " << worldPoint1_ << '\n';
            log_ << "Boundary point 2 was set to: " << worldPoint2_ << '\n';
            log_ << "Boundary point 3 was set to: " << worldPoint3_ << '\n';
            log_ << "Boundary point 4 was set to: " << worldPoint4_ << '\n';
        }
        else if(argument == "-verbose")
        {
            log_.setIsPrintingInCout(true);
        }
        else if (argument == "-logFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Missing argument for '" + argument + "'.");
            }
            logFile_ = std::string(argv[argIndex]);
            std::ofstream testFile(logFile_.c_str());
            if (!testFile.is_open())
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' has a bad value.");
            }
            log_ << "Log file path was set to: " << logFile_ << "\n";
        }
        else if(argument == "-inputFile")
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputFile_ = std::string(argv[argIndex]);
            log_ << "Reading textured mesh from: " << inputFile_ << "\n";
        }
        else if(argument == "-inputGeoRefFile")
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputGeoRefFile_ = std::string(argv[argIndex]);
            log_ << "Reading georef from: " << inputGeoRefFile_ << "\n";
        }
        else if(argument == "-outputFile")
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputFile_ = std::string(argv[argIndex]);
            log_ << "Writing output to: " << outputFile_ << "\n";
        }
        else if(argument == "-outputCornerFile")
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputCornerFile_ = std::string(argv[argIndex]);
            log_ << "Writing corners to: " << outputCornerFile_ << "\n";
        }
        else if(argument == "-inputTransformFile")
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputTransformFile_ = std::string(argv[argIndex]);
            transformOverride_ = true;
            log_ << "Reading transformation matrix from: " << outputCornerFile_ << "\n";
        }
        else
        {
            printHelp();
            throw OdmOrthoPhotoException("Unrecognised argument '" + argument + "'");
        }
    }
    log_ << "\n";
}

void OdmOrthoPhoto::printHelp()
{
    log_.setIsPrintingInCout(true);

    log_ << "OpenDroneMapOrthoPhoto.exe\n\n";

    log_ << "Purpose\n";
    log_ << "Create an orthograpical photo from an oriented textured mesh.\n\n";

    log_ << "Usage:\n";
    log_ << "The program requires a path to an input OBJ mesh file and a resolution, as pixels/m. All other input parameters are optional.\n\n";

    log_ << "The following flags are available\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";

    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable:\n";
    log_ << "\"-inputFile <path>\" (mandatory)\n";
    log_ << "\"Input obj file that must contain a textured mesh.\n\n";

    log_ << "\"-inputGeoRefFile <path>\" (optional, if specified boundary points are assumed to be given as world coordinates. If not specified, the boundary points are assumed to be local coordinates)\n";
    log_ << "\"Input geograpical reference system file that describes the world position of the model's origin.\n\n";

    log_ << "\"-outputFile <path>\" (optional, default: ortho.jpg)\n";
    log_ << "\"Target file in which the orthophoto is saved.\n\n";

    log_ << "\"-outputCornerFile <path>\" (optional)\n";
    log_ << "\"Target text file for boundary corner points, written as \"xmin ymin xmax ymax\".\n\n";

    log_ << "\"-resolution <pixels/m>\" (mandatory)\n";
    log_ << "\"The number of pixels used per meter.\n\n";

    log_ << "\"-boundary <Point1x Point1y Point2x Point2y Point3x Point3y Point4x Point4y>\" (optional, if not specified the entire model will be rendered)\n";
    log_ << "\"Describes the area which should be covered in the ortho photo. The area will be a bounding box containing all four points. The points should be given in the same georeference system as the model.\n\n";

    log_ << "\"-boundaryMinMax <MinX MinY MaxX MaxY>\" (optional, if not specified the entire model will be rendered.)\n";
    log_ << "\"Describes the area which should be covered in the ortho photo. The area will be a bounding box with corners at MinX, MinY and MaxX, MaxY. The points should be given in the same georeference system as the model.\n\n";

    log_.setIsPrintingInCout(false);
}

void OdmOrthoPhoto::createOrthoPhoto()
{
    if(inputFile_.empty())
    {
        throw OdmOrthoPhotoException("Failed to create ortho photo, no texture mesh given.");
    }

    if(boundaryDefined_)
    {
        if(inputGeoRefFile_.empty())
        {
            // Points are assumed to be given in as local points.
            adjustBoundsForLocal();
        }
        else
        {
            // Points are assumed to be given in as world points.
            adjustBoundsForGeoRef();
        }
    }
    else if(!inputGeoRefFile_.empty())
    {
        // No boundary points specified, but georeference system file was given.
        log_ << "Warning:\n";
        log_ << "\tSpecified -inputGeoRefFile, but no boundary points. The georeference system will be ignored.\n";
    }

    log_ << "Reading mesh file...\n";
    // The textured mesh.
    pcl::TextureMesh mesh;
    loadObjFile(inputFile_, mesh);
    log_ << ".. mesh file read.\n\n";

    // Does the model have more than one material?
    multiMaterial_ = 1 < mesh.tex_materials.size();

    bool splitModel = false;

    if(multiMaterial_)
    {
        // Need to check relationship between texture coordinates and faces.
        if(!isModelOk(mesh))
        {
            splitModel = true;
        }
    }

    if(!boundaryDefined_)
    {
        // Determine boundary from model.
        adjustBoundsForEntireModel(mesh);
    }

    // The minimum and maximum boundary values.
    float xMax, xMin, yMax, yMin;
    xMin = std::min(std::min(boundaryPoint1_[0], boundaryPoint2_[0]), std::min(boundaryPoint3_[0], boundaryPoint4_[0]));
    xMax = std::max(std::max(boundaryPoint1_[0], boundaryPoint2_[0]), std::max(boundaryPoint3_[0], boundaryPoint4_[0]));
    yMin = std::min(std::min(boundaryPoint1_[1], boundaryPoint2_[1]), std::min(boundaryPoint3_[1], boundaryPoint4_[1]));
    yMax = std::max(std::max(boundaryPoint1_[1], boundaryPoint2_[1]), std::max(boundaryPoint3_[1], boundaryPoint4_[1]));

    log_ << "Ortho photo bounds x : " << xMin << " -> " << xMax << '\n';
    log_ << "Ortho photo bounds y : " << yMin << " -> " << yMax << '\n';

    // The size of the area.
    float xDiff = xMax - xMin;
    float yDiff = yMax - yMin;
    log_ << "Ortho photo area : " << xDiff*yDiff << "m2\n";

    // The resolution necessary to fit the area with the given resolution.
    int rowRes = static_cast<int>(std::ceil(resolution_*yDiff));
    int colRes = static_cast<int>(std::ceil(resolution_*xDiff));
    log_ << "Ortho photo resolution, width x height : " << colRes << "x" << rowRes << '\n';

    // Check size of photo.
    if(0 >= rowRes*colRes)
    {
        if(0 >= rowRes)
        {
            log_ << "Warning: ortho photo has zero area, height = " << rowRes << ". Forcing height = 1.\n";
            rowRes = 1;
        }
        if(0 >= colRes)
        {
            log_ << "Warning: ortho photo has zero area, width = " << colRes << ". Forcing width = 1.\n";
            colRes = 1;
        }
        log_ << "New ortho photo resolution, width x height : " << colRes << "x" << rowRes << '\n';
    }

    // Init ortho photo
    try{
        photo_ = cv::Mat::zeros(rowRes, colRes, CV_8UC4) + cv::Scalar(255, 255, 255, 0);
        depth_ = cv::Mat::zeros(rowRes, colRes, CV_32F) - std::numeric_limits<float>::infinity();
    }catch(const cv::Exception &e){
        std::cerr << "Couldn't allocate enough memory to render the orthophoto (" << colRes << "x" << rowRes << " cells = " << ((long long)colRes * (long long)rowRes * 4) << " bytes). Try to increase the --orthophoto-resolution parameter to a larger integer or add more RAM.\n";
        exit(1);
    }

    // Contains the vertices of the mesh.
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh.cloud, *meshCloud);

    // Split model and make copies of vertices and texture coordinates for all faces
    if (splitModel)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloudSplit (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > textureCoordinates = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >(0);

        size_t vertexIndexCount = 0;
        for(size_t t = 0; t < mesh.tex_polygons.size(); ++t)
        {
            vertexIndexCount += 3 * mesh.tex_polygons[t].size();
        }
        textureCoordinates.reserve(vertexIndexCount);

        for(size_t t = 0; t < mesh.tex_polygons.size(); ++t)
        {

            for(size_t faceIndex = 0; faceIndex < mesh.tex_polygons[t].size(); ++faceIndex)
            {
                pcl::Vertices polygon = mesh.tex_polygons[t][faceIndex];

                // The index to the vertices of the polygon.
                size_t v1i = polygon.vertices[0];
                size_t v2i = polygon.vertices[1];
                size_t v3i = polygon.vertices[2];

                // The polygon's points.
                pcl::PointXYZ v1 = meshCloud->points[v1i];
                pcl::PointXYZ v2 = meshCloud->points[v2i];
                pcl::PointXYZ v3 = meshCloud->points[v3i];

                Eigen::Vector2f vt1 = mesh.tex_coordinates[0][3*faceIndex];
                Eigen::Vector2f vt2 = mesh.tex_coordinates[0][3*faceIndex + 1];
                Eigen::Vector2f vt3 = mesh.tex_coordinates[0][3*faceIndex + 2];

                meshCloudSplit->points.push_back(v1);
                textureCoordinates.push_back(vt1);
                mesh.tex_polygons[t][faceIndex].vertices[0] = vertexIndexCount;

                meshCloudSplit->points.push_back(v2);
                textureCoordinates.push_back(vt2);
                mesh.tex_polygons[t][faceIndex].vertices[1] = vertexIndexCount;

                meshCloudSplit->points.push_back(v3);
                textureCoordinates.push_back(vt3);
                mesh.tex_polygons[t][faceIndex].vertices[2] = vertexIndexCount;
            }
        }

        mesh.tex_coordinates.clear();
        mesh.tex_coordinates.push_back(textureCoordinates);

        meshCloud = meshCloudSplit;
    }

    // Creates a transformation which aligns the area for the ortho photo.
    Eigen::Transform<float, 3, Eigen::Affine> transform = getROITransform(xMin, -yMax);

    log_ << "Translating and scaling mesh...\n";

    // Move the mesh into position.
    pcl::transformPointCloud(*meshCloud, *meshCloud, transform);
    log_ << ".. mesh translated and scaled.\n\n";

    // Flatten texture coordinates.
    std::vector<Eigen::Vector2f> uvs;
    uvs.reserve(mesh.tex_coordinates.size());
    for(size_t t = 0; t < mesh.tex_coordinates.size(); ++t)
    {
        uvs.insert(uvs.end(), mesh.tex_coordinates[t].begin(), mesh.tex_coordinates[t].end());
    }
    //cv::namedWindow("dsfs");

    // The current material texture
    cv::Mat texture;

    // Used to keep track of the global face index.
    size_t faceOff = 0;

    log_ << "Rendering the ortho photo...\n";

    // Iterate over each part of the mesh (one per material).
    for(size_t t = 0; t < mesh.tex_materials.size(); ++t)
    {
        // The material of the current submesh.
        pcl::TexMaterial material = mesh.tex_materials[t];
        texture = cv::imread(material.tex_file);

        // Check for missing files.
        if(texture.empty())
        {
            log_ << "Material texture could not be read:\n";
            log_ << material.tex_file << '\n';
            log_ << "Could not be read as image, does the file exist?\n";
            continue; // Skip to next material.
        }

        // The faces of the current submesh.
        std::vector<pcl::Vertices> faces = mesh.tex_polygons[t];

        // Iterate over each face...
        for(size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
        {
            // The current polygon.
            pcl::Vertices polygon = faces[faceIndex];

            // ... and draw it into the ortho photo.
            drawTexturedTriangle(texture, polygon, meshCloud, uvs, faceIndex+faceOff);
        }
        faceOff += faces.size();
        log_ << "Material " << t << " rendered.\n";
    }
    log_ << "...ortho photo rendered\n";

    log_ << '\n';
    log_ << "Writing ortho photo to " << outputFile_ << "\n";
    cv::imwrite(outputFile_, photo_);

    if (!outputCornerFile_.empty())
    {
        log_ << "Writing corner coordinates to " << outputCornerFile_ << "\n";
        std::ofstream cornerStream(outputCornerFile_.c_str());
        if (!cornerStream.is_open())
        {
            throw OdmOrthoPhotoException("Failed opening output corner file " + outputCornerFile_ + ".");
        }
        cornerStream.setf(std::ios::scientific, std::ios::floatfield);
        cornerStream.precision(17);
        cornerStream << xMin << " " << yMin << " " << xMax << " " << yMax;
        cornerStream.close();
    }

    log_ << "Orthophoto generation done.\n";
}

void OdmOrthoPhoto::adjustBoundsForGeoRef()
{
    log_ << "Adjusting bounds for world coordinates\n";

    // A stream of the georef system.
    std::ifstream geoRefStream(inputGeoRefFile_.c_str());

    // The system name
    std::string system;
    // The east and north offsets
    int eastOffset, northOffset;

    // Parse file
    std::getline(geoRefStream, system);
    if(!(geoRefStream >> eastOffset))
    {
            throw OdmOrthoPhotoException("Could not extract geographical reference system from \n" + inputGeoRefFile_ + "\nCould not extract east offset.");
    }
    if(!(geoRefStream >> northOffset))
    {
            throw OdmOrthoPhotoException("Could not extract geographical reference system from \n" + inputGeoRefFile_ + "\nCould not extract north offset.");
    }

    log_ << "Georeference system:\n";
    log_ << system << "\n";
    log_ << "East offset: " << eastOffset << "\n";
    log_ << "North offset: " << northOffset << "\n";

    // Adjust boundary points.
    boundaryPoint1_[0] = static_cast<float>(worldPoint1_.eastInteger_  - eastOffset)  + worldPoint1_.eastFractional_;
    boundaryPoint1_[1] = static_cast<float>(worldPoint1_.northInteger_ - northOffset) + worldPoint1_.northFractional_;
    boundaryPoint2_[0] = static_cast<float>(worldPoint2_.eastInteger_  - eastOffset)  + worldPoint2_.eastFractional_;
    boundaryPoint2_[1] = static_cast<float>(worldPoint2_.northInteger_ - northOffset) + worldPoint2_.northFractional_;
    boundaryPoint3_[0] = static_cast<float>(worldPoint3_.eastInteger_  - eastOffset)  + worldPoint3_.eastFractional_;
    boundaryPoint3_[1] = static_cast<float>(worldPoint3_.northInteger_ - northOffset) + worldPoint3_.northFractional_;
    boundaryPoint4_[0] = static_cast<float>(worldPoint4_.eastInteger_  - eastOffset)  + worldPoint4_.eastFractional_;
    boundaryPoint4_[1] = static_cast<float>(worldPoint4_.northInteger_ - northOffset) + worldPoint4_.northFractional_;

    log_ << "Local boundary points:\n";
    log_ << "Point 1: " << boundaryPoint1_[0] << " " << boundaryPoint1_[1] << "\n";
    log_ << "Point 2: " << boundaryPoint2_[0] << " " << boundaryPoint2_[1] << "\n";
    log_ << "Point 3: " << boundaryPoint3_[0] << " " << boundaryPoint3_[1] << "\n";
    log_ << "Point 4: " << boundaryPoint4_[0] << " " << boundaryPoint4_[1] << "\n";
}

void OdmOrthoPhoto::adjustBoundsForLocal()
{
    log_ << "Adjusting bounds for local coordinates\n";

    // Set boundary points from world points.
    boundaryPoint1_[0] = static_cast<float>(worldPoint1_.eastInteger_ )  + worldPoint1_.eastFractional_;
    boundaryPoint1_[1] = static_cast<float>(worldPoint1_.northInteger_) + worldPoint1_.northFractional_;
    boundaryPoint2_[0] = static_cast<float>(worldPoint2_.eastInteger_ )  + worldPoint2_.eastFractional_;
    boundaryPoint2_[1] = static_cast<float>(worldPoint2_.northInteger_) + worldPoint2_.northFractional_;
    boundaryPoint3_[0] = static_cast<float>(worldPoint3_.eastInteger_ )  + worldPoint3_.eastFractional_;
    boundaryPoint3_[1] = static_cast<float>(worldPoint3_.northInteger_) + worldPoint3_.northFractional_;
    boundaryPoint4_[0] = static_cast<float>(worldPoint4_.eastInteger_ )  + worldPoint4_.eastFractional_;
    boundaryPoint4_[1] = static_cast<float>(worldPoint4_.northInteger_) + worldPoint4_.northFractional_;

    log_ << "Local boundary points:\n";
    log_ << "Point 1: " << boundaryPoint1_[0] << " " << boundaryPoint1_[1] << "\n";
    log_ << "Point 2: " << boundaryPoint2_[0] << " " << boundaryPoint2_[1] << "\n";
    log_ << "Point 3: " << boundaryPoint3_[0] << " " << boundaryPoint3_[1] << "\n";
    log_ << "Point 4: " << boundaryPoint4_[0] << " " << boundaryPoint4_[1] << "\n";
    log_ << "\n";
}

void OdmOrthoPhoto::adjustBoundsForEntireModel(const pcl::TextureMesh &mesh)
{
    log_ << "Set boundary to contain entire model.\n";

    // The boundary of the model.
    float xMin, xMax, yMin, yMax;

    xMin = std::numeric_limits<float>::infinity();
    xMax = -std::numeric_limits<float>::infinity();
    yMin = std::numeric_limits<float>::infinity();
    yMax = -std::numeric_limits<float>::infinity();

    // Contains the vertices of the mesh.
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh.cloud, *meshCloud);

    for(size_t t = 0; t < mesh.tex_materials.size(); ++t)
    {
        // The faces of the current submesh.
        std::vector<pcl::Vertices> faces = mesh.tex_polygons[t];

        // Iterate over each face...
        for(size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
        {
            // The current polygon.
            pcl::Vertices polygon = faces[faceIndex];

            // The index to the vertices of the polygon.
            size_t v1i = polygon.vertices[0];
            size_t v2i = polygon.vertices[1];
            size_t v3i = polygon.vertices[2];

            // The polygon's points.
            pcl::PointXYZ v1 = meshCloud->points[v1i];
            pcl::PointXYZ v2 = meshCloud->points[v2i];
            pcl::PointXYZ v3 = meshCloud->points[v3i];

            xMin = std::min(std::min(xMin, v1.x), std::min(v2.x, v3.x));
            xMax = std::max(std::max(xMax, v1.x), std::max(v2.x, v3.x));
            yMin = std::min(std::min(yMin, v1.y), std::min(v2.y, v3.y));
            yMax = std::max(std::max(yMax, v1.y), std::max(v2.y, v3.y));
        }
    }

    // Create dummy boundary points.
    boundaryPoint1_[0] = xMin; boundaryPoint1_[1] = yMin;
    boundaryPoint2_[0] = xMin; boundaryPoint2_[1] = yMax;
    boundaryPoint3_[0] = xMax; boundaryPoint3_[1] = yMax;
    boundaryPoint4_[0] = xMax; boundaryPoint4_[1] = yMin;

    log_ << "Local boundary points:\n";
    log_ << "Point 1: " << boundaryPoint1_[0] << " " << boundaryPoint1_[1] << "\n";
    log_ << "Point 2: " << boundaryPoint2_[0] << " " << boundaryPoint2_[1] << "\n";
    log_ << "Point 3: " << boundaryPoint3_[0] << " " << boundaryPoint3_[1] << "\n";
    log_ << "Point 4: " << boundaryPoint4_[0] << " " << boundaryPoint4_[1] << "\n";
    log_ << "\n";
}

Eigen::Transform<float, 3, Eigen::Affine> OdmOrthoPhoto::getROITransform(float xMin, float yMin) const
{
    //Use transformation matrix if provided:
    if(transformOverride_){
        return readTransform(inputTransformFile_);
    }
    else {
        // The transform used to move the chosen area into the ortho photo.
        Eigen::Transform<float, 3, Eigen::Affine> transform;

        transform(0, 0) = resolution_;     // x Scaling.
        transform(1, 0) = 0.0f;
        transform(2, 0) = 0.0f;
        transform(3, 0) = 0.0f;

        transform(0, 1) = 0.0f;
        transform(1, 1) = -resolution_;     // y Scaling, mirrored for easier rendering.
        transform(2, 1) = 0.0f;
        transform(3, 1) = 0.0f;

        transform(0, 2) = 0.0f;
        transform(1, 2) = 0.0f;
        transform(2, 2) = 1.0f;
        transform(3, 2) = 0.0f;

        transform(0, 3) = -xMin * resolution_;    // x Translation
        transform(1, 3) = -yMin * resolution_;    // y Translation
        transform(2, 3) = 0.0f;
        transform(3, 3) = 1.0f;

        return transform;
    }
}

Eigen::Transform<float, 3, Eigen::Affine> OdmOrthoPhoto::readTransform(std::string transformFile_) const
{
    Eigen::Transform<float, 3, Eigen::Affine> transform;

    std::ifstream transStream(transformFile_.c_str());
    if (!transStream.good())
    {
        throw OdmOrthoPhotoException("Failed opening coordinate file " + transformFile_ + " for reading. " + '\n');
    }

    std::string transString;
    {
        std::getline(transStream, transString);
        std::stringstream l1(transString);
        l1 >> transform(0,0) >> transform(0,1) >> transform(0,2) >> transform(0,3);

        std::getline(transStream, transString);
        std::stringstream l2(transString);
        l2 >> transform(1,0) >> transform(1,1) >> transform(1,2) >> transform(1,3);

        std::getline(transStream, transString);
        std::stringstream l3(transString);
        l3 >> transform(2,0) >> transform(2,1) >> transform(2,2) >> transform(2,3);

        std::getline(transStream, transString);
        std::stringstream l4(transString);
        l4 >> transform(3,0) >> transform(3,1) >> transform(3,2) >> transform(3,3);
    }

    // Don't do any rotation/shear
    transform(0,1) = 0.0f;
    transform(1,0) = 0.0f;

    return transform;
}

void OdmOrthoPhoto::drawTexturedTriangle(const cv::Mat &texture, const pcl::Vertices &polygon, const pcl::PointCloud<pcl::PointXYZ>::Ptr &meshCloud, const std::vector<Eigen::Vector2f> &uvs, size_t faceIndex)
{
    // The index to the vertices of the polygon.
    size_t v1i = polygon.vertices[0];
    size_t v2i = polygon.vertices[1];
    size_t v3i = polygon.vertices[2];

    // The polygon's points.
    pcl::PointXYZ v1 = meshCloud->points[v1i];
    pcl::PointXYZ v2 = meshCloud->points[v2i];
    pcl::PointXYZ v3 = meshCloud->points[v3i];

    if(isSliverPolygon(v1, v2, v3))
    {
        log_ << "Warning: Sliver polygon found at face index " << faceIndex << '\n';
        return;
    }

    // The face data. Position v*{x,y,z}. Texture coordinate v*{u,v}. * is the vertex number in the polygon.
    float v1x, v1y, v1z, v1u, v1v;
    float v2x, v2y, v2z, v2u, v2v;
    float v3x, v3y, v3z, v3u, v3v;

    // Barycentric coordinates of the currently rendered point.
    float l1, l2, l3;

    // The size of the photo, as float.
    float fRows, fCols;
    fRows = static_cast<float>(texture.rows);
    fCols = static_cast<float>(texture.cols);

    // Get vertex position.
    v1x = v1.x; v1y = v1.y; v1z = v1.z;
    v2x = v2.x; v2y = v2.y; v2z = v2.z;
    v3x = v3.x; v3y = v3.y; v3z = v3.z;

    // Get texture coordinates. 
    v1u = uvs[3*faceIndex][0]; v1v = uvs[3*faceIndex][1];
    v2u = uvs[3*faceIndex+1][0]; v2v = uvs[3*faceIndex+1][1];
    v3u = uvs[3*faceIndex+2][0]; v3v = uvs[3*faceIndex+2][1];

    // Check bounding box overlap.
    int xMin = static_cast<int>(std::min(std::min(v1x, v2x), v3x));
    if(xMin > photo_.cols)
    {
        return; // Completely outside to the right.
    }
    int xMax = static_cast<int>(std::max(std::max(v1x, v2x), v3x));
    if(xMax < 0)
    {
        return; // Completely outside to the left.
    }
    int yMin = static_cast<int>(std::min(std::min(v1y, v2y), v3y));
    if(yMin > photo_.rows)
    {
        return; // Completely outside to the top.
    }
    int yMax = static_cast<int>(std::max(std::max(v1y, v2y), v3y));
    if(yMax < 0)
    {
        return; // Completely outside to the bottom.
    }

    // Top point row and column positions
    float topR, topC;
    // Middle point row and column positions
    float midR, midC;
    // Bottom point row and column positions
    float botR, botC;

    // Find top, middle and bottom points.
    if(v1y < v2y)
    {
        if(v1y < v3y)
        {
            if(v2y < v3y)
            {
                // 1 -> 2 -> 3
                topR = v1y; topC = v1x;
                midR = v2y; midC = v2x;
                botR = v3y; botC = v3x;
            }
            else
            {
                // 1 -> 3 -> 2
                topR = v1y; topC = v1x;
                midR = v3y; midC = v3x;
                botR = v2y; botC = v2x;
            }
        }
        else
        {
            // 3 -> 1 -> 2
            topR = v3y; topC = v3x;
            midR = v1y; midC = v1x;
            botR = v2y; botC = v2x;
        }        
    }
    else // v2y <= v1y
    {
        if(v2y < v3y)
        {
            if(v1y < v3y)
            {
                // 2 -> 1 -> 3
                topR = v2y; topC = v2x;
                midR = v1y; midC = v1x;
                botR = v3y; botC = v3x;
            }
            else
            {
                // 2 -> 3 -> 1
                topR = v2y; topC = v2x;
                midR = v3y; midC = v3x;
                botR = v1y; botC = v1x;
            }
        }
        else
        {
            // 3 -> 2 -> 1
            topR = v3y; topC = v3x;
            midR = v2y; midC = v2x;
            botR = v1y; botC = v1x;
        }
    }

    // General appreviations:
    // ---------------------
    // tm : Top(to)Middle.
    // mb : Middle(to)Bottom.
    // tb : Top(to)Bottom.
    // c  : column.
    // r  : row.
    // dr : DeltaRow, step value per row.

    // The step along column for every step along r. Top to middle.
    float ctmdr;
    // The step along column for every step along r. Top to bottom.
    float ctbdr;
    // The step along column for every step along r. Middle to bottom.
    float cmbdr;

    ctbdr = (botC-topC)/(botR-topR);

    // The current column position, from top to middle.
    float ctm = topC;
    // The current column position, from top to bottom.
    float ctb = topC;

    // Check for vertical line between middle and top.
    if(FLT_EPSILON < midR-topR)
    {
        ctmdr = (midC-topC)/(midR-topR);

        // The first pixel row for the bottom part of the triangle.
        int rqStart = std::max(static_cast<int>(std::floor(topR+0.5f)), 0);
        // The last pixel row for the top part of the triangle.
        int rqEnd = std::min(static_cast<int>(std::floor(midR+0.5f)), photo_.rows);

        // Traverse along row from top to middle.
        for(int rq = rqStart; rq < rqEnd; ++rq)
        {
            // Set the current column positions.
            ctm = topC + ctmdr*(static_cast<float>(rq)+0.5f-topR);
            ctb = topC + ctbdr*(static_cast<float>(rq)+0.5f-topR);

            // The first pixel column for the current row.
            int cqStart = std::max(static_cast<int>(std::floor(0.5f+std::min(ctm, ctb))), 0);
            // The last pixel column for the current row.
            int cqEnd = std::min(static_cast<int>(std::floor(0.5f+std::max(ctm, ctb))), photo_.cols);

            for(int cq = cqStart; cq < cqEnd; ++cq)
            {
                // Get barycentric coordinates for the current point.
                getBarycentricCoordinates(v1, v2, v3, static_cast<float>(cq)+0.5f, static_cast<float>(rq)+0.5f, l1, l2, l3);

                if(0.f > l1 || 0.f > l2 || 0.f > l3)
                {
                    //continue;
                }
                
                // The z value for the point.
                float z = v1z*l1+v2z*l2+v3z*l3;

                // Check depth
                float depthValue = depth_.at<float>(rq, cq);
                if(z < depthValue)
                {
                    // Current is behind another, don't draw.
                    continue;
                }

                // The uv values of the point.
                float u, v;
                u = v1u*l1+v2u*l2+v3u*l3;
                v = v1v*l1+v2v*l2+v3v*l3;
                
                renderPixel(rq, cq, u*fCols, (1.0f-v)*fRows, texture);
                
                // Update depth buffer.
                depth_.at<float>(rq, cq) = z;
            }
        }
    }

    if(FLT_EPSILON < botR-midR)
    {
        cmbdr = (botC-midC)/(botR-midR);

        // The current column position, from middle to bottom.
        float cmb = midC;

        // The first pixel row for the bottom part of the triangle.
        int rqStart = std::max(static_cast<int>(std::floor(midR+0.5f)), 0);
        // The last pixel row for the bottom part of the triangle.
        int rqEnd = std::min(static_cast<int>(std::floor(botR+0.5f)), photo_.rows);

        // Traverse along row from middle to bottom.
        for(int rq = rqStart; rq < rqEnd; ++rq)
        {
            // Set the current column positions.
            ctb = topC + ctbdr*(static_cast<float>(rq)+0.5f-topR);
            cmb = midC + cmbdr*(static_cast<float>(rq)+0.5f-midR);

            // The first pixel column for the current row.
            int cqStart = std::max(static_cast<int>(std::floor(0.5f+std::min(cmb, ctb))), 0);
            // The last pixel column for the current row.
            int cqEnd = std::min(static_cast<int>(std::floor(0.5f+std::max(cmb, ctb))), photo_.cols);

            for(int cq = cqStart; cq < cqEnd; ++cq)
            {
                // Get barycentric coordinates for the current point.
                getBarycentricCoordinates(v1, v2, v3, static_cast<float>(cq)+0.5f, static_cast<float>(rq)+0.5f, l1, l2, l3);

                if(0.f > l1 || 0.f > l2 || 0.f > l3)
                {
                    //continue;
                }

                // The z value for the point.
                float z = v1z*l1+v2z*l2+v3z*l3;

                // Check depth
                float depthValue = depth_.at<float>(rq, cq);
                if(z < depthValue)
                {
                    // Current is behind another, don't draw.
                    continue;
                }

                // The uv values of the point.
                float u, v;
                u = v1u*l1+v2u*l2+v3u*l3;
                v = v1v*l1+v2v*l2+v3v*l3;

                renderPixel(rq, cq, u*fCols, (1.0f-v)*fRows, texture);

                // Update depth buffer.
                depth_.at<float>(rq, cq) = z;
            }
        }
    }
}

void OdmOrthoPhoto::renderPixel(int row, int col, float s, float t, const cv::Mat &texture)
{
    // The colors of the texture pixels. tl : top left, tr : top right, bl : bottom left, br : bottom right.
    cv::Vec3b tl, tr, bl, br;
    
    // The offset of the texture coordinate from its pixel positions.
    float leftF, topF;
    // The position of the top left pixel.
    int left, top;
    // The distance to the left and right pixel from the texture coordinate.
    float dl, dt;
    // The distance to the top and bottom pixel from the texture coordinate.
    float dr, db;
    
    dl = modff(s, &leftF);
    dr = 1.0f - dl;
    dt = modff(t, &topF);
    db = 1.0f - dt;
    
    left = static_cast<int>(leftF);
    top = static_cast<int>(topF);
    
    tl = texture.at<cv::Vec3b>(top, left);
    tr = texture.at<cv::Vec3b>(top, left+1);
    bl = texture.at<cv::Vec3b>(top+1, left);
    br = texture.at<cv::Vec3b>(top+1, left+1);
    
    // The interpolated color values.
    float r = 0.0f, g = 0.0f, b = 0.0f;
    
    // Red
    r += static_cast<float>(tl[2]) * dr * db;
    r += static_cast<float>(tr[2]) * dl * db;
    r += static_cast<float>(bl[2]) * dr * dt;
    r += static_cast<float>(br[2]) * dl * dt;
    
    // Green
    g += static_cast<float>(tl[1]) * dr * db;
    g += static_cast<float>(tr[1]) * dl * db;
    g += static_cast<float>(bl[1]) * dr * dt;
    g += static_cast<float>(br[1]) * dl * dt;
    
    // Blue
    b += static_cast<float>(tl[0]) * dr * db;
    b += static_cast<float>(tr[0]) * dl * db;
    b += static_cast<float>(bl[0]) * dr * dt;
    b += static_cast<float>(br[0]) * dl * dt;
    
    photo_.at<cv::Vec4b>(row,col) = cv::Vec4b(static_cast<unsigned char>(b), static_cast<unsigned char>(g), static_cast<unsigned char>(r), 255);
}

void OdmOrthoPhoto::getBarycentricCoordinates(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3, float x, float y, float &l1, float &l2, float &l3) const
{
    // Diff along y.
    float y2y3 = v2.y-v3.y;
    float y1y3 = v1.y-v3.y;
    float y3y1 = v3.y-v1.y;
    float yy3  =  y  -v3.y;
    
    // Diff along x.
    float x3x2 = v3.x-v2.x;
    float x1x3 = v1.x-v3.x;
    float xx3  =  x  -v3.x;
    
    // Normalization factor.
    float norm = (y2y3*x1x3 + x3x2*y1y3);
    
    l1 = (y2y3*(xx3) + x3x2*(yy3)) / norm;
    l2 = (y3y1*(xx3) + x1x3*(yy3)) / norm;
    l3 = 1 - l1 - l2;
}

bool OdmOrthoPhoto::isSliverPolygon(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3) const
{
    // Calculations are made using doubles, to minize rounding errors.
    Eigen::Vector3d a = Eigen::Vector3d(static_cast<double>(v1.x), static_cast<double>(v1.y), static_cast<double>(v1.z));
    Eigen::Vector3d b = Eigen::Vector3d(static_cast<double>(v2.x), static_cast<double>(v2.y), static_cast<double>(v2.z));
    Eigen::Vector3d c = Eigen::Vector3d(static_cast<double>(v3.x), static_cast<double>(v3.y), static_cast<double>(v3.z));
    Eigen::Vector3d dummyVec = (a-b).cross(c-b);

    // Area smaller than, or equal to, floating-point epsilon.
    return std::numeric_limits<float>::epsilon() >= static_cast<float>(std::sqrt(dummyVec.dot(dummyVec))/2.0);
}

bool OdmOrthoPhoto::isModelOk(const pcl::TextureMesh &mesh)
{
    // The number of texture coordinates in the model.
    size_t nTextureCoordinates = 0;
    // The number of faces in the model.
    size_t nFaces = 0;
    
    for(size_t t = 0; t < mesh.tex_coordinates.size(); ++t)
    {
        nTextureCoordinates += mesh.tex_coordinates[t].size();
    }
    for(size_t t = 0; t < mesh.tex_polygons.size(); ++t)
    {
        nFaces += mesh.tex_polygons[t].size();
    }
    
    log_ << "Number of faces in the model " << nFaces << '\n';
    
    return 3*nFaces == nTextureCoordinates;
}


bool OdmOrthoPhoto::loadObjFile(std::string inputFile, pcl::TextureMesh &mesh)
{
    int data_type;
    unsigned int data_idx;
    int file_version;
    int offset = 0;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;

    if (!readHeader(inputFile, mesh.cloud, origin, orientation, file_version, data_type, data_idx, offset))
    {
        throw OdmOrthoPhotoException("Problem reading header in modelfile!\n");
    }

    std::ifstream fs;

    fs.open (inputFile.c_str (), std::ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        //PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
        fs.close ();
        log_<<"Could not read mesh from file ";
        log_ << inputFile.c_str();
        log_ <<"\n";

        throw OdmOrthoPhotoException("Problem reading mesh from file!\n");
    }

    // Seek at the given offset
    fs.seekg (data_idx, std::ios::beg);

    // Get normal_x field indices
    int normal_x_field = -1;
    for (std::size_t i = 0; i < mesh.cloud.fields.size (); ++i)
    {
        if (mesh.cloud.fields[i].name == "normal_x")
        {
            normal_x_field = i;
            break;
        }
    }

    std::size_t v_idx = 0;
    std::size_t vn_idx = 0;
    std::size_t vt_idx = 0;
    std::size_t f_idx = 0;
    std::string line;
    std::vector<std::string> st;
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > coordinates;
    std::vector<Eigen::Vector2f> allTexCoords;

    std::map<int, int> f2vt;

    try
    {
        while (!fs.eof ())
        {
            getline (fs, line);
            // Ignore empty lines
            if (line == "")
                continue;

            // Tokenize the line
            std::stringstream sstream (line);
            sstream.imbue (std::locale::classic ());
            line = sstream.str ();
            boost::trim (line);
            boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

            // Ignore comments
            if (st[0] == "#")
                continue;
            // Vertex
            if (st[0] == "v")
            {
                try
                {
                    for (int i = 1, f = 0; i < 4; ++i, ++f)
                    {
                        float value = boost::lexical_cast<float> (st[i]);
                        memcpy (&mesh.cloud.data[v_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset], &value, sizeof (float));
                    }

                    ++v_idx;
                }
                catch (const boost::bad_lexical_cast &e)
                {
                    log_<<"Unable to convert %s to vertex coordinates!\n";
                    throw OdmOrthoPhotoException("Unable to convert %s to vertex coordinates!");
                }
                continue;
            }
            // Vertex normal
            if (st[0] == "vn")
            {
                try
                {
                    for (int i = 1, f = normal_x_field; i < 4; ++i, ++f)
                    {
                        float value = boost::lexical_cast<float> (st[i]);
                        memcpy (&mesh.cloud.data[vn_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
                        &value,
                        sizeof (float));
                    }
                    ++vn_idx;
                }
                catch (const boost::bad_lexical_cast &e)
                {
                    log_<<"Unable to convert %s to vertex normal!\n";
                    throw OdmOrthoPhotoException("Unable to convert %s to vertex normal!");
                }
                continue;
            }
            // Texture coordinates
            if (st[0] == "vt")
            {
                try
                {
                    Eigen::Vector3f c (0, 0, 0);
                    for (std::size_t i = 1; i < st.size (); ++i)
                        c[i-1] = boost::lexical_cast<float> (st[i]);

                    if (c[2] == 0)
                        coordinates.push_back (Eigen::Vector2f (c[0], c[1]));
                    else
                        coordinates.push_back (Eigen::Vector2f (c[0]/c[2], c[1]/c[2]));
                    ++vt_idx;

                }
                catch (const boost::bad_lexical_cast &e)
                {
                    log_<<"Unable to convert %s to vertex texture coordinates!\n";
                    throw OdmOrthoPhotoException("Unable to convert %s to vertex texture coordinates!");
                }
                continue;
            }
            // Material
            if (st[0] == "usemtl")
            {
                mesh.tex_polygons.push_back (std::vector<pcl::Vertices> ());
                mesh.tex_materials.push_back (pcl::TexMaterial ());
                for (std::size_t i = 0; i < companions_.size (); ++i)
                {
                    std::vector<pcl::TexMaterial>::const_iterator mat_it = companions_[i].getMaterial (st[1]);
                    if (mat_it != companions_[i].materials_.end ())
                    {
                        mesh.tex_materials.back () = *mat_it;
                        break;
                    }
                }
                // We didn't find the appropriate material so we create it here with name only.
                if (mesh.tex_materials.back ().tex_name == "")
                    mesh.tex_materials.back ().tex_name = st[1];
                mesh.tex_coordinates.push_back (coordinates);
                coordinates.clear ();
                continue;
            }
            // Face
            if (st[0] == "f")
            {
                //We only care for vertices indices
                pcl::Vertices face_v; face_v.vertices.resize (st.size () - 1);
                for (std::size_t i = 1; i < st.size (); ++i)
                {
                    int v;
                    sscanf (st[i].c_str (), "%d", &v);
                    v = (v < 0) ? v_idx + v : v - 1;
                    face_v.vertices[i-1] = v;

                    int v2, vt, vn;
                    sscanf (st[i].c_str (), "%d/%d/%d", &v2, &vt, &vn);
                    f2vt[3*(f_idx) + i-1] = vt-1;
                }
                mesh.tex_polygons.back ().push_back (face_v);
                ++f_idx;
                continue;
            }
        }
    }
    catch (const char *exception)
    {
        fs.close ();
        log_<<"Unable to read file!\n";
        throw OdmOrthoPhotoException("Unable to read file!");
    }

    if (vt_idx != v_idx)
    {
        std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texcoordinates = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >(0);
        texcoordinates.reserve(3*f_idx);

        for (size_t faceIndex = 0; faceIndex < f_idx; ++faceIndex)
        {
            for(size_t i = 0; i < 3; ++i)
            {
                Eigen::Vector2f vt = mesh.tex_coordinates[0][f2vt[3*faceIndex+i]];
                texcoordinates.push_back(vt);
            }
        }

        mesh.tex_coordinates.clear();
        mesh.tex_coordinates.push_back(texcoordinates);
    }

    fs.close();
    return (0);
}

bool OdmOrthoPhoto::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                                Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                                int &file_version, int &data_type, unsigned int &data_idx,
                                const int offset)
{
    origin       = Eigen::Vector4f::Zero ();
    orientation  = Eigen::Quaternionf::Identity ();
    file_version = 0;
    cloud.width  = cloud.height = cloud.point_step = cloud.row_step = 0;
    cloud.data.clear ();
    data_type = 0;
    data_idx = offset;

    std::ifstream fs;
    std::string line;

    if (file_name == "" || !boost::filesystem::exists (file_name))
    {
        return false;
    }

    // Open file in binary mode to avoid problem of
    // std::getline() corrupting the result of ifstream::tellg()
    fs.open (file_name.c_str (), std::ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        fs.close ();
        return false;
    }

    // Seek at the given offset
    fs.seekg (offset, std::ios::beg);

    // Read the header and fill it in with wonderful values
    bool vertex_normal_found = false;
    bool vertex_texture_found = false;
    // Material library, skip for now!
    // bool material_found = false;
    std::vector<std::string> material_files;
    std::size_t nr_point = 0;
    std::vector<std::string> st;

    try
    {
        while (!fs.eof ())
        {
            getline (fs, line);
            // Ignore empty lines
            if (line == "")
            continue;

            // Tokenize the line
            std::stringstream sstream (line);
            sstream.imbue (std::locale::classic ());
            line = sstream.str ();
            boost::trim (line);
            boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
            // Ignore comments
            if (st.at (0) == "#")
                continue;

            // Vertex
            if (st.at (0) == "v")
            {
                ++nr_point;
                continue;
            }

            // Vertex texture
            if ((st.at (0) == "vt") && !vertex_texture_found)
            {
                vertex_texture_found = true;
                continue;
            }

            // Vertex normal
            if ((st.at (0) == "vn") && !vertex_normal_found)
            {
                vertex_normal_found = true;
                continue;
            }

            // Material library, skip for now!
            if (st.at (0) == "mtllib")
            {
                material_files.push_back (st.at (1));
                continue;
            }
        }
    }
    catch (const char *exception)
    {
        fs.close ();
        return false;
    }

    if (!nr_point)
    {
        fs.close ();
        return false;
    }

    int field_offset = 0;
    for (int i = 0; i < 3; ++i, field_offset += 4)
    {
        cloud.fields.push_back (pcl::PCLPointField ());
        cloud.fields[i].offset   = field_offset;
        cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
        cloud.fields[i].count    = 1;
    }

    cloud.fields[0].name = "x";
    cloud.fields[1].name = "y";
    cloud.fields[2].name = "z";

    if (vertex_normal_found)
    {
        std::string normals_names[3] = { "normal_x", "normal_y", "normal_z" };
        for (int i = 0; i < 3; ++i, field_offset += 4)
        {
            cloud.fields.push_back (pcl::PCLPointField ());
            pcl::PCLPointField& last = cloud.fields.back ();
            last.name     = normals_names[i];
            last.offset   = field_offset;
            last.datatype = pcl::PCLPointField::FLOAT32;
            last.count    = 1;
        }
    }

    if (material_files.size () > 0)
    {
        for (std::size_t i = 0; i < material_files.size (); ++i)
        {
            pcl::MTLReader companion;

            if (companion.read (file_name, material_files[i]))
            {
                log_<<"Problem reading material file.";
            }

            companions_.push_back (companion);
        }
    }

    cloud.point_step = field_offset;
    cloud.width      = nr_point;
    cloud.height     = 1;
    cloud.row_step   = cloud.point_step * cloud.width;
    cloud.is_dense   = true;
    cloud.data.resize (cloud.point_step * nr_point);
    fs.close ();
    return true;
}
