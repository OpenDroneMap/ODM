// PCL
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>

// Modified PCL
#include "modifiedPclFunctions.hpp"

// This
#include "Georef.hpp"

std::ostream& operator<<(std::ostream &os, const GeorefSystem &geo)
{
    return os << geo.system_ << " " << static_cast<int>(geo.falseEasting_) << " " << static_cast<int>(geo.falseNorthing_);
}

GeorefCamera::GeorefCamera()
    :focalLength_(0.0), k1_(0.0), k2_(0.0), transform_(NULL), position_(NULL)
{
}

GeorefCamera::GeorefCamera(const GeorefCamera &other)
    : focalLength_(other.focalLength_), k1_(other.k1_), k2_(other.k2_),
      easting_(other.easting_), northing_(other.northing_), altitude_(other.altitude_), 
      transform_(NULL), position_(NULL)
{
    if(NULL != other.transform_)
    {
        transform_ = new Eigen::Affine3f(*other.transform_);
    }
    if(NULL != other.position_)
    {
        position_ = new Eigen::Vector3f(*other.position_);
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
}

void GeorefCamera::extractCamera(std::ifstream &bundleStream)
{
    // Extract intrinsic parameters.
    bundleStream >> focalLength_ >> k1_ >> k2_;
    
    Eigen::Vector3f t;
    Eigen::Matrix3f rot;
    Eigen::Affine3f transform;
    
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

Georef::Georef()
{
    log_.setIsPrintingInCout(true);
    
    bundleFilename_ = "";
    coordFilename_ = "";
    inputObjFilename_ = "";
    outputObjFilename_ = "";
}

Georef::~Georef()
{
}

int Georef::run(int argc, char *argv[])
{
    try
    {
        parseArguments(argc, argv);
        makeGeoreferencedModel();
    }
    catch (const GeorefException& e)
    {
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_ << "Error in Georef:\n";
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (...)
    {
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
        else if(argument == "-coordFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw GeorefException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            coordFilename_ = std::string(argv[argIndex]);
            log_ << "Reading cameras georeferenced positions from: " << coordFilename_ << "\n";
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
        else
        {
            printHelp();
            throw GeorefException("Unrecognised argument '" + argument + "'");
        }
    }
    
    if(!outputSpecified)
    {
        makeDefaultOutput();
    }
}

void Georef::printHelp()
{
    bool printInCoutPop = log_.isPrintingInCout();
    log_.setIsPrintingInCout(true);
    
    log_ << "Georef.exe\n\n";
    
    log_ << "Purpose:" << "\n";
    log_ << "Create an orthograpical photo from an oriented textured mesh." << "\n";
    
    log_ << "Usage:" << "\n";
    log_ << "The program requires a path to a camera bundle file, a camera georeference coords file, and an input OBJ mesh file. All other input parameters are optional." << "\n\n";
    
    log_ << "The following flags are available\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";
    
    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: " << "\n";
    log_ << "\"-bundleFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input cameras bundle file.\n\n";
    
    log_ << "\"-coordFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input cameras geroreferenced coords file.\n\n";
    
    log_ << "\"-inputFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input obj file that must contain a textured mesh.\n\n";
    
    log_ << "\"-outputFile <path>\" (optional, default <inputFile>_geo)" << "\n";
    log_ << "\"Output obj file that will contain the georeferenced texture mesh.\n\n";
    
    log_.setIsPrintingInCout(printInCoutPop);
}

void Georef::makeDefaultOutput()
{
    if(inputObjFilename_.empty())
    {
        throw GeorefException("Tried to generate default ouptut file without having an input file.");
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

void Georef::makeGeoreferencedModel()
{
    // Read translations from bundle file
    std::ifstream bundleStream(bundleFilename_.c_str());
    if (!bundleStream.good())
    {
        throw GeorefException("Failed opening " + bundleFilename_ + " for reading." + '\n');
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
    
    // Read coords from coord file generated by extract_utm tool
    std::ifstream coordStream(coordFilename_.c_str());
    if (!coordStream.good())
    {
        throw GeorefException("Failed opening " + coordFilename_ + " for reading." + '\n');
    }
    
    std::string coordString;
    std::getline(coordStream, georefSystem_.system_); // System
    {
        std::getline(coordStream, coordString); // Flase easting & northing.
        std::stringstream ss(coordString);
        
        ss >> georefSystem_.falseEasting_ >> georefSystem_.falseNorthing_;
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
            throw GeorefException("Error to many cameras in \'" + coordFilename_ + "\' coord file.\n");
        }
        
        std::istringstream istr(coordString);
        cameras_[nGeorefCameras].extractCameraGeoref(istr);
        
        ++nGeorefCameras;
    }
    coordStream.close();
    
    if(nGeorefCameras < cameras_.size())
    {
        throw GeorefException("Not enough cameras in \'" + coordFilename_ + "\' coord file.\n");
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
    log_ << "Reading mesh file...\n";
    // The textureds mesh.e
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
    
    printGeorefSystem();
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
    
    tmp = tmp.substr(0, findPos);
    
    tmp = tmp + "_georef_system.txt";
    log_ << '\n';
    log_ << "Saving georeference system file to \'" << tmp << "\'...\n";
    std::ofstream geoStream(tmp.c_str());
    geoStream << georefSystem_ << std::endl;
    geoStream.close();
    log_ << "... georeference system saved.\n";
}

