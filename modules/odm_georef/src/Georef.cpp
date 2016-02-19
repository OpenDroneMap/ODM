// PCL
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Modified PCL
#include "modifiedPclFunctions.hpp"

// This
#include "Georef.hpp"

std::ostream& operator<<(std::ostream &os, const GeorefSystem &geo)
{
    return os << geo.system_ << "\n" << static_cast<int>(geo.eastingOffset_) << " " << static_cast<int>(geo.northingOffset_);
}

GeorefGCP::GeorefGCP()
    :x_(0.0), y_(0.0), z_(0.0), use_(false), localX_(0.0), localY_(0.0), localZ_(0.0),cameraIndex_(0), pixelX_(0), pixelY_(0.0), image_("")
{
}

GeorefGCP::~GeorefGCP()
{
}

void GeorefGCP::extractGCP(std::istringstream &gcpStream)
{
    gcpStream >> x_ >> y_ >> z_ >> pixelX_ >> pixelY_ >> image_;
}

Vec3 GeorefGCP::getPos()
{
    return Vec3(localX_,localY_,localZ_);
}

Vec3 GeorefGCP::getReferencedPos()
{
    return Vec3(x_,y_,z_);
}

GeorefCamera::GeorefCamera()
    :focalLength_(0.0), k1_(0.0), k2_(0.0), transform_(NULL), position_(NULL), pose_(NULL)
{
}

GeorefCamera::GeorefCamera(const GeorefCamera &other)
    : focalLength_(other.focalLength_), k1_(other.k1_), k2_(other.k2_),
      easting_(other.easting_), northing_(other.northing_), altitude_(other.altitude_), 
      transform_(NULL), position_(NULL), pose_(NULL)
{
    if(NULL != other.transform_)
    {
        transform_ = new Eigen::Affine3f(*other.transform_);
    }
    if(NULL != other.position_)
    {
        position_ = new Eigen::Vector3f(*other.position_);
    }
    if(pose_ != other.pose_)
    {
        pose_ = new Eigen::Affine3f(*other.pose_);
    }
}

GeorefCamera::~GeorefCamera()
{
    if(NULL != transform_)
    {
        delete transform_;
        transform_ = NULL;
    }
    if(NULL != position_)
    {
        delete position_;
        position_ = NULL;
    }
    if(pose_ != NULL)
    {
        delete pose_;
        pose_ = NULL;
    }
}

void GeorefCamera::extractCamera(std::ifstream &bundleStream)
{
    // Extract intrinsic parameters.
    bundleStream >> focalLength_ >> k1_ >> k2_;
    
    Eigen::Vector3f t;
    Eigen::Matrix3f rot;
    Eigen::Affine3f transform;
    Eigen::Affine3f pose;
    
    bundleStream >> transform(0,0); // Read rotation (0,0) from bundle file
    bundleStream >> transform(0,1); // Read rotation (0,1) from bundle file
    bundleStream >> transform(0,2); // Read rotation (0,2) from bundle file
    
    bundleStream >> transform(1,0); // Read rotation (1,0) from bundle file
    bundleStream >> transform(1,1); // Read rotation (1,1) from bundle file
    bundleStream >> transform(1,2); // Read rotation (1,2) from bundle file
    
    bundleStream >> transform(2,0); // Read rotation (2,0) from bundle file
    bundleStream >> transform(2,1); // Read rotation (2,1) from bundle file
    bundleStream >> transform(2,2); // Read rotation (2,2) from bundle file
    
    bundleStream >> t(0); // Read translation (0,3) from bundle file
    bundleStream >> t(1); // Read translation (1,3) from bundle file
    bundleStream >> t(2); // Read translation (2,3) from bundle file
    
    //
    pose(0,0) = transform(0,0);
    pose(0,1) = transform(0,1);
    pose(0,2) = transform(0,2);

    pose(1,0) = transform(1,0);
    pose(1,1) = transform(1,1);
    pose(1,2) = transform(1,2);

    pose(2,0) = transform(2,0);
    pose(2,1) = transform(2,1);
    pose(2,2) = transform(2,2);

    pose(0,3) = t(0);
    pose(1,3) = t(1);
    pose(2,3) = t(2);

    pose(3,0) = 0.0;
    pose(3,1) = 0.0;
    pose(3,2) = 0.0;
    pose(3,3) = 1.0;

    pose = pose.inverse();

    // Column negation
    pose(0,2) = -1.0*pose(0,2);
    pose(1,2) = -1.0*pose(1,2);
    pose(2,2) = -1.0*pose(2,2);

    pose(0,1) = -1.0*pose(0,1);
    pose(1,1) = -1.0*pose(1,1);
    pose(2,1) = -1.0*pose(2,1);

    if (pose_ != NULL)
    {
        delete pose_;
        pose_ = NULL;
    }

    pose_ = new Eigen::Affine3f(pose);

    rot = transform.matrix().topLeftCorner<3,3>();
    
    // Calculate translation according to -R't and store in vector.
    t = -rot.transpose()*t;
    
    transform(0,3) = t(0);
    transform(1,3) = t(1);
    transform(2,3) = t(2);


    // Set transform and position.
    if(NULL != transform_)
    {
        delete transform_;
        transform_ = NULL;
    }
    
    transform_ = new Eigen::Affine3f(transform);
    
    if(NULL != position_)
    {
        delete position_;
        position_ = NULL;
    }
    position_ = new Eigen::Vector3f(t);
}

void GeorefCamera::extractCameraGeoref(std::istringstream &coordStream)
{
    coordStream >> easting_ >> northing_ >> altitude_;
}

Vec3 GeorefCamera::getPos()
{
    return Vec3((*position_)(0),(*position_)(1),(*position_)(2));
}

Vec3 GeorefCamera::getReferencedPos()
{
    return Vec3(easting_,northing_,altitude_);
}

std::ostream& operator<<(std::ostream &os, const GeorefCamera &cam)
{
    os << "Focal, k1, k2 : " << cam.focalLength_ << ", " << cam.k1_ << ", " << cam.k2_ << "\n";
    if(NULL != cam.transform_)
    {
        os << "Transform :\n" << cam.transform_->matrix() << "\n";
    }
    else
    {
        os << "Transform :\nNULL\n";
    }
    if(NULL != cam.position_)
    {
        os << "Position :\n" << cam.position_->matrix() << "\n";
    }
    else
    {
        os << "Position :\nNULL\n";
    }
    os << "east, north, alt : " << cam.easting_ << ", " << cam.northing_ << ", " << cam.altitude_ << '\n';
    return os;
}

Georef::Georef() : log_(false)
{
    georeferencePointCloud_ = false;
    useGCP_ = false;
    bundleFilename_ = "";
    inputCoordFilename_ = "";
    outputCoordFilename_ = "";
    inputObjFilename_ = "";
    outputObjFilename_ = "";
    exportCoordinateFile_ = false;
    exportGeorefSystem_ = false;
}

Georef::~Georef()
{
}

int Georef::run(int argc, char *argv[])
{
    try
    {
        parseArguments(argc, argv);
        createGeoreferencedModel();
    }
    catch (const GeorefException& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in Georef:\n";
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

void Georef::parseArguments(int argc, char *argv[])
{
    bool outputSpecified = false;
    bool outputPointCloudSpecified = false;
    bool imageListSpecified = false;
    bool gcpFileSpecified = false;
    bool imageLocation = false;
    bool bundleResized = false;
    bool outputCoordSpecified = false;
    bool inputCoordSpecified = false;
    
    logFile_ = std::string(argv[0]) + "_log.txt";
    log_ << logFile_ << "\n";
    
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
        else if(argument == "-verbose")
        {
            log_.setIsPrintingInCout(true);
        }
        else if (argument == "-logFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw GeorefException("Missing argument for '" + argument + "'.");
            }
            logFile_ = std::string(argv[argIndex]);
            std::ofstream testFile(logFile_.c_str());
            if (!testFile.is_open())
            {
                throw GeorefException("Argument '" + argument + "' has a bad value.");
            }
            log_ << "Log file path was set to: " << logFile_ << "\n";
        }
        else if(argument == "-bundleFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            bundleFilename_ = std::string(argv[argIndex]);
            log_ << "Reading cameras from: " << bundleFilename_ << "\n";
        }
        else if(argument == "-inputCoordFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputCoordFilename_ = std::string(argv[argIndex]);
            log_ << "Reading cameras gps exif positions from: " << inputCoordFilename_ << "\n";
            inputCoordSpecified = true;
        }
        else if(argument == "-outputCoordFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputCoordFilename_ = std::string(argv[argIndex]);
            log_ << "Exporting cameras georeferenced gps positions to: " << outputCoordFilename_ << "\n";
            exportCoordinateFile_ = true;
            outputCoordSpecified = true;
        }
        else if(argument == "-inputFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputObjFilename_ = std::string(argv[argIndex]);
            log_ << "Reading textured mesh from: " << inputObjFilename_ << "\n";
        }
        else if(argument == "-inputPointCloudFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputPointCloudFilename_ = std::string(argv[argIndex]);
            log_ << "Reading point cloud from: " << inputPointCloudFilename_ << "\n";
            georeferencePointCloud_ = true;
        }
        else if(argument == "-gcpFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            gcpFilename_ = std::string(argv[argIndex]);
            log_ << "Reading GCPs from: " << gcpFilename_ << "\n";
            gcpFileSpecified = true;
        }
        else if(argument == "-imagesListPath" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            imagesListPath_ = std::string(argv[argIndex]);
            log_ << "Reading image list from: " << imagesListPath_ << "\n";
            imageListSpecified = true;
        }
        else if(argument == "-imagesPath" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            imagesLocation_ = std::string(argv[argIndex]);
            log_ << "Images location is set to: " << imagesLocation_ << "\n";
            imageLocation = true;
        }
        else if(argument == "-georefFileOutputPath" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            georefFilename_ = std::string(argv[argIndex]);
            log_ << "Georef file output path is set to: " << georefFilename_ << "\n";
            exportGeorefSystem_ = true;
        }
        else if(argument == "-bundleResizedTo" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> bundleResizedTo_;
            if (ss.bad())
            {
                throw GeorefException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "Bundle resize value is set to: " << bundleResizedTo_ << "\n";
            bundleResized = true;
        }
        else if(argument == "-outputFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputObjFilename_ = std::string(argv[argIndex]);
            log_ << "Writing output to: " << outputObjFilename_ << "\n";
            outputSpecified = true;
        }
        else if(argument == "-outputPointCloudFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputPointCloudFilename_ = std::string(argv[argIndex]);
            log_ << "Writing output to: " << outputPointCloudFilename_ << "\n";
            outputPointCloudSpecified = true;
        }
        else
        {
            printHelp();
            throw GeorefException("Unrecognised argument '" + argument + "'");
        }
    }
    
    if (inputCoordSpecified && outputCoordSpecified)
    {
        throw GeorefException("Both output and input coordfile specified, only one of those are accepted.");
    }

    if (imageListSpecified && gcpFileSpecified && imageLocation && bundleResized)
    {
        useGCP_ = true;
    }
    else
    {
        log_ << '\n';
        log_ << "Missing input in order to use GCP for georeferencing. Using EXIF data instead.\n";
    }

    if(georeferencePointCloud_ && !outputPointCloudSpecified)
    {
        setDefaultPointCloudOutput();
    }

    if(!outputSpecified)
    {
        setDefaultOutput();
    }
}

void Georef::printHelp()
{
    bool printInCoutPop = log_.isPrintingInCout();
    log_.setIsPrintingInCout(true);
    
    log_ << "Georef.exe\n\n";
    
    log_ << "Purpose:" << "\n";
    log_ << "Georeference a textured mesh with the use of ground control points or exif data from the images." << "\n";
    
    log_ << "Usage:" << "\n";
    log_ << "The program requires a path to a camera bundle file, a camera georeference coords file, and an input OBJ mesh file. All other input parameters are optional." << "\n\n";
    
    log_ << "The following flags are available\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";
    
    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: " << "\n";
    log_ << "\"-bundleFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input cameras bundle file.\n\n";

    log_ << "\"-gcpFile <path>\" (mandatory if using ground control points)\n";
    log_ << "Path to the file containing the ground control points used for georeferencing.\n";
    log_ << "The file needs to be on the following line format:\n";
    log_ << "easting northing height pixelrow pixelcol imagename\n\n";
    
    log_ << "\"-inputCoordFile <path>\" (mandatory if using exif data)" << "\n";
    log_ << "\"Input cameras geroreferenced coords file.\n\n";

    log_ << "\"-outputCoordFile <path>\" (optional)" << "\n";
    log_ << "\"Output cameras geroreferenced coords file.\n\n";
    
    log_ << "\"-inputFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input obj file that must contain a textured mesh.\n\n";

    log_ << "\"-inputPointCloudFile <path>\" (optional)" << "\n";
    log_ << "\"Input ply file that must contain a point cloud.\n\n";

    log_ << "\"-imagesListPath <path>\" (mandatory if using ground control points)\n";
    log_ << "Path to the list containing the image names used in the bundle.out file.\n\n";

    log_ << "\"-imagesPath <path>\" (mandatory if using ground control points)\n";
    log_ << "Path to the folder containing full resolution images.\n\n";

    log_ << "\"-bundleResizedTo <integer>\" (mandatory if using ground control points)\n";
    log_ << "The resized resolution used in bundler.\n\n";
    
    log_ << "\"-outputFile <path>\" (optional, default <inputFile>_geo)" << "\n";
    log_ << "\"Output obj file that will contain the georeferenced texture mesh.\n\n";

    log_ << "\"-outputPointCloudFile <path>\" (mandatory if georeferencing a point cloud)" << "\n";
    log_ << "\"Output ply file that will contain the georeferenced point cloud.\n\n";
    
    log_.setIsPrintingInCout(printInCoutPop);
}

void Georef::setDefaultOutput()
{
    if(inputObjFilename_.empty())
    {
        throw GeorefException("Tried to generate default output file without having an input file.");
    }
    
    std::string tmp = inputObjFilename_;
    size_t findPos = tmp.find_last_of(".");
    
    if(std::string::npos == findPos)
    {
        throw GeorefException("Tried to generate default ouptut file, could not find .obj in the input file:\n\'"+inputObjFilename_+"\'");
    }
    
    tmp = tmp.substr(0, findPos);

    outputObjFilename_ = tmp + "_geo.obj";
    log_ << "Writing output to: " << outputObjFilename_ << "\n";
}

void Georef::setDefaultPointCloudOutput()
{
    if(inputPointCloudFilename_.empty())
    {
        throw GeorefException("Tried to generate default point cloud ouptut file without having an input file.");
    }

    std::string tmp = inputPointCloudFilename_;
    size_t findPos = tmp.find_last_of(".");

    if(std::string::npos == findPos)
    {
        throw GeorefException("Tried to generate default ouptut file, could not find .ply in the input file:\n\'"+inputPointCloudFilename_+"\'");
    }

    tmp = tmp.substr(0, findPos);

    outputPointCloudFilename_ = tmp + "_geo.ply";
    log_ << "Writing output to: " << outputPointCloudFilename_ << "\n";
}

void Georef::createGeoreferencedModel()
{
    if (useGCP_)
    {
        createGeoreferencedModelFromGCPData();
    }
    else
    {
        createGeoreferencedModelFromExifData();
    }
}

void Georef::readCameras()
{
    // Read translations from bundle file
    std::ifstream bundleStream(bundleFilename_.c_str());
    if (!bundleStream.good())
    {
        throw GeorefException("Failed opening bundle file " + bundleFilename_ + " for reading." + '\n');
    }

    // Read Cameras.
    std::string bundleLine;
    std::getline(bundleStream, bundleLine); // Read past bundle version comment
    int numCameras, numPoints;
    bundleStream >> numCameras >> numPoints;
    for (int i=0; i<numCameras; ++i)
    {
        cameras_.push_back(GeorefCamera());
        cameras_.back().extractCamera(bundleStream);
    }
}

void Georef::readGCPs()
{
    std::ifstream imageListStream(imagesListPath_.c_str());
    if (!imageListStream.good())
    {
        throw GeorefException("Failed opening image path " + imagesListPath_ + " for reading.\n");
    }

    for (size_t i=0; i<cameras_.size(); ++i)
    {
        std::string imageName;
        imageListStream >> imageName;
        imageList_.push_back(imageName);
    }

    // Number of GCPs read
    size_t nrGCPs = 0;

    std::ifstream gcpStream(gcpFilename_.c_str());
    if (!gcpStream.good())
    {
        throw GeorefException("Failed opening gcp file " + gcpFilename_ + " for reading.\n");
    }
    std::string gcpString;

    // Read the first line in the file as the format of the projected coordinates
    std::getline(gcpStream, georefSystem_.system_);

    log_ << '\n';
    log_<< "Reading following GCPs from file:\n";

    // Read all GCPs
    while(std::getline(gcpStream, gcpString))
    {
        std::istringstream istr(gcpString);
        GeorefGCP gcp;
        gcp.extractGCP(istr);
        gcps_.push_back(gcp);
        ++nrGCPs;

        log_<<"x_: "<<gcp.x_<<" y_: "<<gcp.y_<<" z_: "<<gcp.z_<<" pixelX_: "<<gcp.pixelX_<<" pixelY_: "<<gcp.pixelY_<<" image: "<<gcp.image_<<"\n";
    }

    // Check if the GCPs have corresponding images in the bundle files and if they don't, remove them from the GCP-list
    for (size_t gcpIndex = 0; gcpIndex<gcps_.size(); ++gcpIndex)
    {
        bool imageExists = false;
        for (size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
        {
            size_t found = imageList_[cameraIndex].find(gcps_[gcpIndex].image_);
            if (found != std::string::npos)
            {
                gcps_[gcpIndex].cameraIndex_ = cameraIndex;
                imageExists = true;
            }
        }
        if (!imageExists)
        {
            log_ <<"Can't find image "<<gcps_[gcpIndex].image_<<". The corresponding GCP will not be used for georeferencing.\n";
            gcps_.erase(gcps_.begin() + gcpIndex);
            --gcpIndex;
        }
    }
}

void Georef::calculateGCPOffset()
{
    // Offsets
    double eastingOffset = 0;
    double northingOffset = 0;

    // Add all GCPs to weight an offset
    for (size_t gcpIndex = 0; gcpIndex<gcps_.size(); ++gcpIndex)
    {
        eastingOffset += (gcps_[gcpIndex].x_)/static_cast<double>(gcps_.size());
        northingOffset += (gcps_[gcpIndex].y_)/static_cast<double>(gcps_.size());
    }

    georefSystem_.eastingOffset_ = static_cast<int>(std::floor(eastingOffset));
    georefSystem_.northingOffset_ = static_cast<int>(std::floor(northingOffset));

    log_ << '\n';
    log_<<"The calculated easting offset for the georeferenced system: "<<georefSystem_.eastingOffset_<<"\n";
    log_<<"The calculated northing offset for the georeferenced system: "<<georefSystem_.northingOffset_<<"\n";

    log_ << '\n';
    log_ << "Recalculated GCPs with offset:\n";

    // Subtract the offset from all GCPs
    for (size_t gcpIndex = 0; gcpIndex<gcps_.size(); ++gcpIndex)
    {
        gcps_[gcpIndex].x_ -= static_cast<double>(georefSystem_.eastingOffset_);
        gcps_[gcpIndex].y_ -= static_cast<double>(georefSystem_.northingOffset_);
        log_<<"x_: "<<gcps_[gcpIndex].x_<<" y_: "<<gcps_[gcpIndex].y_<<" z_: "<<gcps_[gcpIndex].z_<<"\n";
    }
}

pcl::PointXYZ Georef::barycentricCoordinates(pcl::PointXY point, pcl::PointXYZ vert0, pcl::PointXYZ vert1, pcl::PointXYZ vert2, pcl::PointXY p0, pcl::PointXY p1, pcl::PointXY p2)
{
    // Shorthands
    double x0 = p0.x; double y0 = p0.y;
    double x1 = p1.x; double y1 = p1.y;
    double x2 = p2.x; double y2 = p2.y;
    double x = point.x; double y = point.y;

    double q1x = x1 - x0;
    double q1y = y1 - y0;
    double q2x = x2 - x0;
    double q2y = y2 - y0;

    double norm = q1x * q2y - q1y * q2x;
    double l1 = q2y*(x - x0) - q2x*(y - y0);
    l1 /= norm;
    double l2 = -q1y*(x - x0) + q1x*(y - y0);
    l2 /= norm;

    pcl::PointXYZ res;
    res.x = (1.0 - l1 - l2)*vert0.x + l1*vert1.x + l2*vert2.x;
    res.y = (1.0 - l1 - l2)*vert0.y + l1*vert1.y + l2*vert2.y;
    res.z = (1.0 - l1 - l2)*vert0.z + l1*vert1.z + l2*vert2.z;

    return res;
}

void Georef::performGeoreferencingWithGCP()
{
    log_ << '\n';
    log_ << "Reading mesh file " << inputObjFilename_ <<"\n";
    log_ << '\n';
    pcl::TextureMesh mesh;
    if (pcl::io::loadOBJFile(inputObjFilename_, mesh) == -1)
    {
        throw GeorefException("Error when reading model from:\n" + inputObjFilename_ + "\n");
    }
    else
    {
        log_ << "Successfully loaded " << inputObjFilename_ << ".\n";
    }

    // Convert vertices to pcl::PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh.cloud, *meshCloud);

    // The number of GCP that is usable
    int nrGCPUsable = 0;

    for (size_t gcpIndex = 0; gcpIndex < gcps_.size(); ++gcpIndex)
    {
        // Bool to check if the GCP is intersecting any triangle
        bool exists = false;

        // Translate the GeoreferenceCamera to pcl-format in order to use pcl-functions
        pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
        cam.focal_length = cameras_[gcps_[gcpIndex].cameraIndex_].focalLength_;
        cam.pose = *(cameras_[gcps_[gcpIndex].cameraIndex_].pose_);
        cam.texture_file = imagesLocation_ + '/' + gcps_[gcpIndex].image_;

        cv::Mat image = cv::imread(cam.texture_file);
        cam.height = static_cast<double>(image.rows);
        cam.width = static_cast<double>(image.cols);
        cam.focal_length *= static_cast<double>(cam.width)/bundleResizedTo_;

        // The pixel position for the GCP in pcl-format in order to use pcl-functions
        pcl::PointXY gcpPos;
        gcpPos.x = static_cast<float>(gcps_[gcpIndex].pixelX_);
        gcpPos.y = static_cast<float>(gcps_[gcpIndex].pixelY_);

        // Move vertices in mesh into the camera coordinate system
        pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*meshCloud, *cameraCloud, cam.pose.inverse());

        // The vertex indicies to be used in order to calculate the GCP in the models coordinates
        size_t vert0Index = 0; size_t vert1Index = 0; size_t vert2Index = 0;

        pcl::PointXY bestPixelPos0; pcl::PointXY bestPixelPos1; pcl::PointXY bestPixelPos2;

        // The closest distance of a triangle to the camera
        double bestDistance = std::numeric_limits<double>::infinity();

        // Loop through all submeshes in model
        for (size_t meshIndex = 0; meshIndex < mesh.tex_polygons.size(); ++meshIndex)
        {
            // Loop through all faces in submesh and check if inside polygon
            for (size_t faceIndex = 0; faceIndex < mesh.tex_polygons[meshIndex].size(); ++faceIndex)
            {
                // Variables for the vertices in face as projections in the camera plane
                pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;
                if (isFaceProjected(cam,
                                    cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[0]],
                                    cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[1]],
                                    cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[2]],
                                    pixelPos0, pixelPos1, pixelPos2))
                {
                    // If the pixel position of the GCP is inside the current triangle
                    if (checkPointInsideTriangle(pixelPos0, pixelPos1, pixelPos2, gcpPos))
                    {
                        // Extract distances for all vertices for face to camera
                        double d0 = cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[0]].z;
                        double d1 = cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[1]].z;
                        double d2 = cameraCloud->points[mesh.tex_polygons[meshIndex][faceIndex].vertices[2]].z;

                        // Calculate largest distance and store in distance variable
                        double distance = std::max(d0, std::max(d1,d2));

                        // If the triangle is closer to the camera use this triangle
                        if (distance < bestDistance)
                        {
                            // Update variables for the closest polygon
                            bestDistance = distance;
                            vert0Index = mesh.tex_polygons[meshIndex][faceIndex].vertices[0];
                            vert1Index = mesh.tex_polygons[meshIndex][faceIndex].vertices[1];
                            vert2Index = mesh.tex_polygons[meshIndex][faceIndex].vertices[2];
                            bestPixelPos0 = pixelPos0;
                            bestPixelPos1 = pixelPos1;
                            bestPixelPos2 = pixelPos2;
                            exists = true;
                            ++nrGCPUsable;
                        }
                    }
                }
            }
        }

        if(exists)
        {
            // Shorthands for the vertices
            pcl::PointXYZ v0 = meshCloud->points[vert0Index];
            pcl::PointXYZ v1 = meshCloud->points[vert1Index];
            pcl::PointXYZ v2 = meshCloud->points[vert2Index];
            // Use barycentric coordinates to calculate position for the polygon intersection
            pcl::PointXYZ gcpLocal = barycentricCoordinates(gcpPos, v0, v1, v2, bestPixelPos0, bestPixelPos1, bestPixelPos2);

            log_ << "Position in model for gcp " << gcpIndex + 1<< ": x=" <<gcpLocal.x<<" y="<<gcpLocal.y<<" z="<<gcpLocal.z<<"\n";
            gcps_[gcpIndex].localX_ = gcpLocal.x;
            gcps_[gcpIndex].localY_ = gcpLocal.y;
            gcps_[gcpIndex].localZ_ = gcpLocal.z;
            gcps_[gcpIndex].use_ = true;
        }
    }

    if (nrGCPUsable < 3)
    {
        throw GeorefException("Less than 3 GCPs have correspondences in the generated model.");
    }

    size_t gcp0; size_t gcp1; size_t gcp2;
    log_ << '\n';
    log_ << "Choosing optimal gcp triplet...\n";
    chooseBestGCPTriplet(gcp0, gcp1, gcp2);
    log_ << "Optimal gcp triplet chosen: ";
    log_ << gcp0 << ", " << gcp1 << ", " << gcp2 << '\n';
    log_ << '\n';
    FindTransform transFinal;
    transFinal.findTransform(gcps_[gcp0].getPos(), gcps_[gcp1].getPos(), gcps_[gcp2].getPos(),
                             gcps_[gcp0].getReferencedPos(), gcps_[gcp1].getReferencedPos(), gcps_[gcp2].getReferencedPos());
    log_ << "Final transform:\n";
    log_ << transFinal.transform_ << '\n';

    // The tranform used to transform model into the georeferenced system.
    Eigen::Transform<float, 3, Eigen::Affine> transform;

    transform(0, 0) = static_cast<float>(transFinal.transform_.r1c1_);
    transform(1, 0) = static_cast<float>(transFinal.transform_.r2c1_);
    transform(2, 0) = static_cast<float>(transFinal.transform_.r3c1_);
    transform(3, 0) = static_cast<float>(transFinal.transform_.r4c1_);

    transform(0, 1) = static_cast<float>(transFinal.transform_.r1c2_);
    transform(1, 1) = static_cast<float>(transFinal.transform_.r2c2_);
    transform(2, 1) = static_cast<float>(transFinal.transform_.r3c2_);
    transform(3, 1) = static_cast<float>(transFinal.transform_.r4c2_);

    transform(0, 2) = static_cast<float>(transFinal.transform_.r1c3_);
    transform(1, 2) = static_cast<float>(transFinal.transform_.r2c3_);
    transform(2, 2) = static_cast<float>(transFinal.transform_.r3c3_);
    transform(3, 2) = static_cast<float>(transFinal.transform_.r4c3_);

    transform(0, 3) = static_cast<float>(transFinal.transform_.r1c4_);
    transform(1, 3) = static_cast<float>(transFinal.transform_.r2c4_);
    transform(2, 3) = static_cast<float>(transFinal.transform_.r3c4_);
    transform(3, 3) = static_cast<float>(transFinal.transform_.r4c4_);

    log_ << '\n';
    log_ << "Applying transform to mesh...\n";
    // Move the mesh into position.
    pcl::transformPointCloud(*meshCloud, *meshCloud, transform);
    log_ << ".. mesh transformed.\n";

    // Update the mesh.
    pcl::toPCLPointCloud2 (*meshCloud, mesh.cloud);

    // Iterate over each part of the mesh (one per material), to make texture file paths relative the .mtl file.
    for(size_t t = 0; t < mesh.tex_materials.size(); ++t)
    {
        // The material of the current submesh.
        pcl::TexMaterial& material = mesh.tex_materials[t];

        size_t find = material.tex_file.find_last_of("/\\");
        if(std::string::npos != find)
        {
            material.tex_file = material.tex_file.substr(find + 1);
        }
    }

    log_ << '\n';
    if (saveOBJFile(outputObjFilename_, mesh, 8) == -1)
    {
        throw GeorefException("Error when saving model:\n" + outputObjFilename_ + "\n");
    }
    else
    {
        log_ << "Successfully saved model.\n";
    }

    if(georeferencePointCloud_)
    {
        //pcl::PointCloud2<pcl::PointNormal>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> (inputPointCloudFilename_.c_str(), *pointCloud.get()) == -1) {
            throw GeorefException("Error when reading point cloud:\n" + inputPointCloudFilename_ + "\n");
        }
        else
        {
            log_ << "Successfully loaded " << pointCloud->size() << " points with corresponding normals from file.\n";
        }
        log_ << '\n';
        log_ << "Applying transform to point cloud...\n";
        pcl::transformPointCloud(*pointCloud, *pointCloud, transform);
        log_ << ".. point cloud transformed.\n";

        pcl::PLYWriter plyWriter;

        log_ << '\n';
        log_ << "Saving point cloud file to \'" << outputPointCloudFilename_ << "\'...\n";
        //pcl::io::savePLYFileASCII(outputPointCloudFilename_.c_str(), *pointCloud.get());
        plyWriter.write(outputPointCloudFilename_.c_str(), *pointCloud.get(), false, false);
        log_ << ".. point cloud file saved.\n";
    }

    if(exportCoordinateFile_)
    {
        log_ << '\n';
        log_ << "Saving georeferenced camera positions to ";
        log_ << outputCoordFilename_;
        log_<< "\n";
        std::ofstream coordStream(outputCoordFilename_.c_str());
        coordStream << georefSystem_.system_ <<std::endl;
        coordStream << static_cast<int>(georefSystem_.eastingOffset_) << " " << static_cast<int>(georefSystem_.northingOffset_) << std::endl;
        for(size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
        {
            Vec3 globalCameraPosition = (transFinal.transform_)*(cameras_[cameraIndex].getPos());
            coordStream << globalCameraPosition.x_ << " " << globalCameraPosition.y_ << " " << globalCameraPosition.z_ << std::endl;
        }
        coordStream.close();
        log_ << "...coordinate file saved.\n";
    }

    if(exportGeorefSystem_)
    {
        printGeorefSystem();
    }
}

void Georef::createGeoreferencedModelFromGCPData()
{
    readCameras();

    readGCPs();

    calculateGCPOffset();

    performGeoreferencingWithGCP();

}

void Georef::createGeoreferencedModelFromExifData()
{
    readCameras();
    
    // Read coords from coord file generated by extract_utm tool
    std::ifstream coordStream(inputCoordFilename_.c_str());
    if (!coordStream.good())
    {
        throw GeorefException("Failed opening coordinate file " + inputCoordFilename_ + " for reading." + '\n');
    }
    
    std::string coordString;
    std::getline(coordStream, georefSystem_.system_); // System
    {
        std::getline(coordStream, coordString);
        std::stringstream ss(coordString);
        
        ss >> georefSystem_.eastingOffset_ >> georefSystem_.northingOffset_;
    }
    
    log_ << '\n';
    log_ << "Geographical reference system\n";
    log_ << georefSystem_ << '\n';
    
    // The number of cameras in the coords file.
    size_t nGeorefCameras = 0;
    
    // Read the georefernced position for all cameras.
    while (std::getline(coordStream, coordString))
    {
        if(nGeorefCameras >= cameras_.size())
        {
            throw GeorefException("Error, to many cameras in \'" + inputCoordFilename_ + "\' coord file.\n");
        }
        
        std::istringstream istr(coordString);
        cameras_[nGeorefCameras].extractCameraGeoref(istr);
        
        ++nGeorefCameras;
    }
    coordStream.close();
    
    if(nGeorefCameras < cameras_.size())
    {
        throw GeorefException("Not enough cameras in \'" + inputCoordFilename_ + "\' coord file.\n");
    }
    
    // The optimal camera triplet.
    size_t cam0, cam1, cam2;
    
    log_ << '\n';
    log_ << "Choosing optimal camera triplet...\n";
    chooseBestCameraTriplet(cam0, cam1, cam2);
    log_ << "... optimal camera triplet chosen:\n";    
    log_ << cam0 << ", " << cam1 << ", " << cam2 << '\n';
    log_ << '\n';
    FindTransform transFinal;
    transFinal.findTransform(cameras_[cam0].getPos(), cameras_[cam1].getPos(), cameras_[cam2].getPos(),
                             cameras_[cam0].getReferencedPos(), cameras_[cam1].getReferencedPos(), cameras_[cam2].getReferencedPos());
    log_ << "Final transform:\n";
    log_ << transFinal.transform_ << '\n';
    
    // The tranform used to move the chosen area into the ortho photo.
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    
    transform(0, 0) = static_cast<float>(transFinal.transform_.r1c1_);    transform(1, 0) = static_cast<float>(transFinal.transform_.r2c1_);
    transform(2, 0) = static_cast<float>(transFinal.transform_.r3c1_);    transform(3, 0) = static_cast<float>(transFinal.transform_.r4c1_);
    
    transform(0, 1) = static_cast<float>(transFinal.transform_.r1c2_);    transform(1, 1) = static_cast<float>(transFinal.transform_.r2c2_);
    transform(2, 1) = static_cast<float>(transFinal.transform_.r3c2_);    transform(3, 1) = static_cast<float>(transFinal.transform_.r4c2_);
    
    transform(0, 2) = static_cast<float>(transFinal.transform_.r1c3_);    transform(1, 2) = static_cast<float>(transFinal.transform_.r2c3_);
    transform(2, 2) = static_cast<float>(transFinal.transform_.r3c3_);    transform(3, 2) = static_cast<float>(transFinal.transform_.r4c3_);
    
    transform(0, 3) = static_cast<float>(transFinal.transform_.r1c4_);    transform(1, 3) = static_cast<float>(transFinal.transform_.r2c4_);
    transform(2, 3) = static_cast<float>(transFinal.transform_.r3c4_);    transform(3, 3) = static_cast<float>(transFinal.transform_.r4c4_);
    
    log_ << '\n';
    log_ << "Reading mesh file...\n";
    pcl::TextureMesh mesh;
    pcl::io::loadOBJFile(inputObjFilename_, mesh);
    log_ << ".. mesh file read.\n";
    
    // Contains the vertices of the mesh.
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh.cloud, *meshCloud);
    
    log_ << '\n';
    log_ << "Applying transform to mesh...\n";
    // Move the mesh into position.
    pcl::transformPointCloud(*meshCloud, *meshCloud, transform);
    log_ << ".. mesh transformed.\n";
    
    // Update the mesh.
    pcl::toPCLPointCloud2 (*meshCloud, mesh.cloud);
    
    // Iterate over each part of the mesh (one per material), to make texture file paths relative the .mtl file.
    for(size_t t = 0; t < mesh.tex_materials.size(); ++t)
    {
        // The material of the current submesh.
        pcl::TexMaterial& material = mesh.tex_materials[t];
        
        size_t find = material.tex_file.find_last_of("/\\");
        if(std::string::npos != find)
        {
            material.tex_file = material.tex_file.substr(find + 1);
        }
    }
    
    log_ << '\n';
    log_ << "Saving mesh file to \'" << outputObjFilename_ << "\'...\n";
    saveOBJFile(outputObjFilename_, mesh, 8);
    log_ << ".. mesh file saved.\n";
    
    if(georeferencePointCloud_)
    {
        //pcl::PointCloud2<pcl::PointNormal>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBNormal> (inputPointCloudFilename_.c_str(), *pointCloud.get()) == -1) {
            throw GeorefException("Error when reading point cloud:\n" + inputPointCloudFilename_ + "\n");
        }
        else
        {
            log_ << "Successfully loaded " << pointCloud->size() << " points with corresponding normals from file.\n";
        }
        log_ << '\n';
        log_ << "Applying transform to point cloud...\n";
        pcl::transformPointCloud(*pointCloud, *pointCloud, transform);
        log_ << ".. point cloud transformed.\n";

        pcl::PLYWriter plyWriter;

        log_ << '\n';
        log_ << "Saving point cloud file to \'" << outputPointCloudFilename_ << "\'...\n";
        //pcl::io::savePLYFileASCII(outputPointCloudFilename_.c_str(), *pointCloud.get());
        plyWriter.write(outputPointCloudFilename_.c_str(), *pointCloud.get(), false, false);
        log_ << ".. point cloud file saved.\n";
    }

    if(exportGeorefSystem_)
    {
        printGeorefSystem();
    }
}

void Georef::chooseBestGCPTriplet(size_t &gcp0, size_t &gcp1, size_t &gcp2)
{
    double minTotError = std::numeric_limits<double>::infinity();

    for(size_t t = 0; t < gcps_.size(); ++t)
    {
        if (gcps_[t].use_)
        {
            for(size_t s = t; s < gcps_.size(); ++s)
            {
                if (gcps_[s].use_)
                {
                    for(size_t p = s; p < gcps_.size(); ++p)
                    {
                        if (gcps_[p].use_)
                        {
                            FindTransform trans;
                            trans.findTransform(gcps_[t].getPos(), gcps_[s].getPos(), gcps_[p].getPos(),
                                                gcps_[t].getReferencedPos(), gcps_[s].getReferencedPos(), gcps_[p].getReferencedPos());

                            // The total error for the curren camera triplet.
                            double totError = 0.0;

                            for(size_t r = 0; r < gcps_.size(); ++r)
                            {
                                totError += trans.error(gcps_[r].getPos(), gcps_[r].getReferencedPos());
                            }

                            if(minTotError > totError)
                            {
                                minTotError = totError;
                                gcp0 = t;
                                gcp1 = s;
                                gcp2 = p;
                            }
                        }
                    }
                }
            }
        }
    }
    log_ << "Mean georeference error " << minTotError / static_cast<double>(cameras_.size()) << '\n';
}

void Georef::chooseBestCameraTriplet(size_t &cam0, size_t &cam1, size_t &cam2)
{
    double minTotError = std::numeric_limits<double>::infinity();
    
    for(size_t t = 0; t < cameras_.size(); ++t)
    {
        for(size_t s = t; s < cameras_.size(); ++s)
        {
            for(size_t p = s; p < cameras_.size(); ++p)
            {
                FindTransform trans;
                trans.findTransform(cameras_[t].getPos(), cameras_[s].getPos(), cameras_[p].getPos(),
                                    cameras_[t].getReferencedPos(), cameras_[s].getReferencedPos(), cameras_[p].getReferencedPos());
                
                // The total error for the curren camera triplet.
                double totError = 0.0;
                
                for(size_t r = 0; r < cameras_.size(); ++r)
                {
                    totError += trans.error(cameras_[r].getPos(), cameras_[r].getReferencedPos());
                }
                
                if(minTotError > totError)
                {
                    minTotError = totError;
                    cam0 = t;
                    cam1 = s;
                    cam2 = p;
                }
            }
        }
    }
    
    log_ << "Mean georeference error " << minTotError / static_cast<double>(cameras_.size()) << '\n';
}

void Georef::printGeorefSystem()
{
    if(outputObjFilename_.empty())
    {
        throw GeorefException("Output file path empty!.");
    }
    
    std::string tmp = outputObjFilename_;
    size_t findPos = tmp.find_last_of(".");
    
    if(std::string::npos == findPos)
    {
        throw GeorefException("Tried to generate default ouptut file, could not find .obj in the output file:\n\'"+outputObjFilename_+"\'");
    }
    
    //tmp = tmp.substr(0, findPos);
    
    //tmp = tmp + "_georef_system.txt";
    log_ << '\n';
    log_ << "Saving georeference system file to \'" << georefFilename_ << "\'...\n";
    std::ofstream geoStream(georefFilename_.c_str());
    geoStream << georefSystem_ << std::endl;
    geoStream.close();
    log_ << "... georeference system saved.\n";
}


