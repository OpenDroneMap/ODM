

// This
#include "OdmOrthoPhoto.hpp"

OdmOrthoPhoto::OdmOrthoPhoto()
    :log_(false)
{
    inputFile_ = "";
    outputFile_ = "ortho.jpg";
    logFile_    = "log.txt";
    
    resolution_ = 0.0f;
    boundryPoint1_[0] = 0.0f; boundryPoint1_[1] = 0.0f;
    boundryPoint2_[0] = 0.0f; boundryPoint2_[1] = 0.0f;
    boundryPoint3_[0] = 0.0f; boundryPoint3_[1] = 0.0f;
    boundryPoint4_[0] = 0.0f; boundryPoint4_[1] = 0.0f;
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
        log_ << e.what() << "\n";
        log_.print(logFile_);
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_ << "Error in OdmOrthoPhoto:\n";
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

void OdmOrthoPhoto::parseArguments(int argc, char *argv[])
{
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
        else if(argument == "-boundry")
        {
            if(argIndex+8 >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 8 more input following it, but no more inputs were provided.");
            }
            
            std::stringstream ss;
            ss << argv[argIndex+1] << " " << argv[argIndex+2] << " " << argv[argIndex+3] << " " << argv[argIndex+4] << " " << argv[argIndex+5] << " " << argv[argIndex+6] << " " << argv[argIndex+7] << " " << argv[argIndex+8];
            ss >> boundryPoint1_[0] >> boundryPoint1_[1] >> boundryPoint2_[0] >> boundryPoint2_[1] >> boundryPoint3_[0] >> boundryPoint3_[1] >> boundryPoint4_[0] >> boundryPoint4_[1];
            argIndex += 8;
            
            log_ << "Boundry point 1 was set to: " << boundryPoint1_[0] << ", " << boundryPoint1_[1] << '\n';
            log_ << "Boundry point 2 was set to: " << boundryPoint2_[0] << ", " << boundryPoint2_[1] << '\n';
            log_ << "Boundry point 3 was set to: " << boundryPoint3_[0] << ", " << boundryPoint3_[1] << '\n';
            log_ << "Boundry point 4 was set to: " << boundryPoint4_[0] << ", " << boundryPoint4_[1] << '\n';
        }
        else if(argument == "-verbose")
        {
            log_.setIsPrintingInCout(true);
        }
        else if(argument == "-inputFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            inputFile_ = std::string(argv[argIndex]);
            log_ << "Reading textured mesh from: " << inputFile_ << "\n";
        }
        else if(argument == "-outputFile" && argIndex < argc)
        {
            argIndex++;
            if (argIndex >= argc)
            {
                throw OdmOrthoPhotoException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            }
            outputFile_ = std::string(argv[argIndex]);
            log_ << "Writing output to: " << outputFile_ << "\n";
        }
        else
        {
            printHelp();
            throw OdmOrthoPhotoException("Unrecognised argument '" + argument + "'");
        }
    }
}

void OdmOrthoPhoto::printHelp()
{
    log_.setIsPrintingInCout(true);
    
    log_ << "OpenDroneMapOrthoPhoto.exe\n\n";
    
    log_ << "Purpose:" << "\n";
    log_ << "Create an orthograpical photo from an oriented textured mesh." << "\n";
    
    log_ << "Usage:" << "\n";
    log_ << "The program requires a path to an input OBJ mesh file, resolution and boundry points to define the area. All other input parameters are optional." << "\n\n";
    
    log_ << "The following flags are available\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";
    
    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: " << "\n";
    log_ << "\"-inputFile <path>\" (mandatory)" << "\n";
    log_ << "\"Input obj file that must contain a textured mesh.\n\n";
    
    log_ << "\"-outputFile <path>\" (optional, default: ortho.jpg)" << "\n";
    log_ << "\"Target file in which the orthophoto is saved.\n\n";
    
    log_ << "\"-resolution <pixels/m>\" (mandatory)" << "\n";
    log_ << "\"The number of pixels used per meter.\n\n";
    
    log_ << "\"-boundry <Point1x Point1y Point2x Point2y Point3x Point3y Point4x Point4y >\" (mandatory)" << "\n";
    log_ << "\"Describes the area which should be covered in the ortho photo. The area will be a bounding box containing all four points.\n\n";
    
    log_.setIsPrintingInCout(false);
}

void OdmOrthoPhoto::createOrthoPhoto()
{
    if(inputFile_.empty())
    {
        throw OdmOrthoPhotoException("Failed to create ortho photo, no texture mesh given.");
        return;
    }
    
    log_ << '\n';
    log_ << "Reading mesh file...\n";
    // The textureds mesh.
    pcl::TextureMesh mesh;
    pcl::io::loadOBJFile(inputFile_, mesh);
    log_ << ".. mesh file read.\n";
    
    // Does the model have more than one material?
    multiMaterial_ = 1 < mesh.tex_materials.size();
    
    if(multiMaterial_)
    {
        // Need to check relationship between texture coordinates and faces.
        if(!isModelOk(mesh))
        {
            throw OdmOrthoPhotoException("Could not generate ortho photo: The given mesh has multiple textures, but the number of texture coordinates is NOT equal to 3 times the number of faces.");
        }
    }
    
    // The minimum and maximum boundry values.
    float xMax, xMin, yMax, yMin;
    xMin = std::min(std::min(boundryPoint1_[0], boundryPoint2_[0]), std::min(boundryPoint3_[0], boundryPoint4_[0]));
    xMax = std::max(std::max(boundryPoint1_[0], boundryPoint2_[0]), std::max(boundryPoint3_[0], boundryPoint4_[0]));
    yMin = std::min(std::min(boundryPoint1_[1], boundryPoint2_[1]), std::min(boundryPoint3_[1], boundryPoint4_[1]));
    yMax = std::max(std::max(boundryPoint1_[1], boundryPoint2_[1]), std::max(boundryPoint3_[1], boundryPoint4_[1]));
    
    log_ << "Ortho photo area x : " << xMin << " -> " << xMax << '\n';
    log_ << "Ortho photo area y : " << yMin << " -> " << yMax << '\n';
    
    // The size of the area.
    float xDiff = xMax - xMin;
    float yDiff = yMax - yMin;
    
    // The resolution neccesary to fit the area with the given resolution.
    int rowRes = static_cast<int>(std::ceil(resolution_*xDiff));
    int colRes = static_cast<int>(std::ceil(resolution_*yDiff));
    log_ << "Ortho photo resolution, width x height : " << colRes << "x" << rowRes << '\n';
    
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
    photo_ = cv::Mat::zeros(rowRes, colRes, CV_8UC3) + cv::Scalar(255, 255, 255);
    depth_ = cv::Mat::zeros(rowRes, colRes, CV_32F) - std::numeric_limits<float>::infinity();
    
    // Contains the vertices of the mesh.
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh.cloud, *meshCloud);
    
    // Creates a transformation which aligns the area for the ortho photo.
    Eigen::Transform<float, 3, Eigen::Affine> transform = getROITransform(xMin, -yMax);
    
    log_ << "Translating and scaling mesh...\n";
    
    // Move the mesh into position.
    pcl::transformPointCloud(*meshCloud, *meshCloud, transform);
    log_ << ".. mesh translated and scaled.\n";
    
    // Flatten texture coordiantes.
    std::vector<Eigen::Vector2f> uvs;
    for(size_t t = 0; t < mesh.tex_coordinates.size(); ++t)
    {
        uvs.insert(uvs.end(), mesh.tex_coordinates[t].begin(), mesh.tex_coordinates[t].end());
    }
    
    // The current material texture
    cv::Mat texture;
    
    // Used to keep track of the global face index.
    size_t faceOff = 0;
    
    log_ << '\n';
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
    log_ << "Orthophoto generation done.\n";
}

Eigen::Transform<float, 3, Eigen::Affine> OdmOrthoPhoto::getROITransform(float xMin, float yMin) const
{
    // The tranform used to move the chosen area into the ortho photo.
    Eigen::Transform<float, 3, Eigen::Affine> transform;
    
    transform(0, 0) = resolution_;
    transform(1, 0) = 0.0f;
    transform(2, 0) = 0.0f;
    transform(3, 0) = 0.0f;
    
    transform(0, 1) = 0.0f;
    transform(1, 1) = -resolution_;
    transform(2, 1) = 0.0f;
    transform(3, 1) = 0.0f;
    
    transform(0, 2) = 0.0f;
    transform(1, 2) = 0.0f;
    transform(2, 2) = 1.0f;
    transform(3, 2) = 0.0f;
    
    transform(0, 3) = -xMin*resolution_;
    transform(1, 3) = -yMin*resolution_;
    transform(2, 3) = 0.0f;
    transform(3, 3) = 1.0f;
    
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
    
    // Get texture coorinates.
    if(multiMaterial_)
    {
        v1u = uvs[3*faceIndex][0]; v1v = uvs[3*faceIndex][1];
        v2u = uvs[3*faceIndex+1][0]; v2v = uvs[3*faceIndex+1][1];
        v3u = uvs[3*faceIndex+2][0]; v3v = uvs[3*faceIndex+2][1];
    }
    else
    {
        v1u = uvs[v1i][0]; v1v = uvs[v1i][1];
        v2u = uvs[v2i][0]; v2v = uvs[v2i][1];
        v3u = uvs[v3i][0]; v3v = uvs[v3i][1];
    }
    
    // Check bounding box overlap.
    int xMin = static_cast<int>(std::min(std::min(v1x, v2x), v3x));
    if(xMin > photo_.cols)
    {
        return; // Outside to the right.
    }
    int xMax = static_cast<int>(std::max(std::max(v1x, v2x), v3x));
    if(xMax < 0)
    {
        return; // Outside to the left.
    }
    int yMin = static_cast<int>(std::min(std::min(v1y, v2y), v3y));
    if(yMin > photo_.rows)
    {
        return; // Outside to the top.
    }
    int yMax = static_cast<int>(std::max(std::max(v1y, v2y), v3y));
    if(yMax < 0)
    {
        return; // Outside to the bottom.
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
        
        // Describes the offset from the pixel coordiante to the triangle point.
        float rRoundOff = topR - std::floor(topR);
        
        // Travers along row from top to middle.
        for(int rq = static_cast<int>(std::floor(topR))+1; rq <= static_cast<int>(std::floor(midR)); ++rq)
        {
            // Set the current column positions.
            ctm = topC + ctmdr*(static_cast<float>(rq)-topR);
            ctb = topC + ctbdr*(static_cast<float>(rq)-topR);
            
            // Describes the offset from the pixel coordiante to the triangle point.
            float cRoundOff = std::min(ctm, ctb) - std::floor(std::min(ctm, ctb));
            
            for(int cq = static_cast<int>(std::floor(std::min(ctm, ctb))); cq < static_cast<int>(std::floor(std::max(ctm, ctb))); ++cq)
            {
                if(0 <= rq && rq < photo_.rows && 0 <= cq && cq < photo_.cols)
                {
                    // Get barycentric coordinates for the current point.
                    getBarycentricCoordiantes(v1, v2, v3, static_cast<float>(cq)+cRoundOff, static_cast<float>(rq)+rRoundOff, l1, l2, l3);
                    
                    // The z value for the point.
                    float z = v1z*l1+v2z*l2+v3z*l3;
                    
                    // Check depth
                    float depthValue = depth_.at<float>(rq, cq);
                    if(z < depthValue)
                    {
                        // Current is behind last, don't draw.
                        continue;
                    }
                    
                    // The uv values of the point.
                    float u, v;
                    u = v1u*l1+v2u*l2+v3u*l3;
                    v = v1v*l1+v2v*l2+v3v*l3;
                    
                    // Nearest neighbour texture interpolation.
                    int uq = static_cast<int>(u*fCols);
                    int vq = static_cast<int>((1.0f-v)*fRows);
                    photo_.at<cv::Vec3b>(rq,cq) = texture.at<cv::Vec3b>(vq, uq);
                    
                    // Update depth buffer.
                    depth_.at<float>(rq, cq) = z;
                }
            }
        }
    }
    
    if(FLT_EPSILON < botR-midR)
    {
        cmbdr = (botC-midC)/(botR-midR);
        
        // The current column position, from middle to bottom.
        float cmb = midC;
        
        // Describes the offset from the pixel coordiante to the triangle point.
        float rRoundOff = midR - std::floor(midR);
        
        // Travers along row from middle to bottom.
        for(int rq = static_cast<int>(std::floor(midR))+1; rq <= static_cast<int>(std::floor(botR)); ++rq)
        {
            // Set the current column positions.
            ctb = topC + ctbdr*(static_cast<float>(rq)-topR);
            cmb = midC + cmbdr*(static_cast<float>(rq)-midR);
            
            // Describes the offset from the pixel coordiante to the triangle point.
            float cRoundOff = std::min(cmb, ctb) - std::floor(std::min(cmb, ctb));
            
            for(int cq = static_cast<int>(std::floor(std::min(cmb, ctb))); cq < static_cast<int>(std::floor(std::max(cmb, ctb))); ++cq)
            {
                if(0 <= rq && rq < photo_.rows && 0 <= cq && cq < photo_.cols)
                {
                    // Get barycentric coordinates for the current point.
                    getBarycentricCoordiantes(v1, v2, v3, static_cast<float>(cq)+cRoundOff, static_cast<float>(rq)+rRoundOff, l1, l2, l3);
                    
                    // The z value for the point.
                    float z = v1z*l1+v2z*l2+v3z*l3;
                    
                    // Check depth
                    float depthValue = depth_.at<float>(rq, cq);
                    if(z < depthValue)
                    {
                        // Current is behind last, don't draw.
                        continue;
                    }
                    
                    // The uv values of the point.
                    float u, v;
                    u = v1u*l1+v2u*l2+v3u*l3;
                    v = v1v*l1+v2v*l2+v3v*l3;
                    
                    // Nearest neighbour texture interpolation.
                    int uq = static_cast<int>(u*fCols);
                    int vq = static_cast<int>((1.0f-v)*fRows);
                    photo_.at<cv::Vec3b>(rq,cq) = texture.at<cv::Vec3b>(vq, uq);
                    
                    // Update depth buffer.
                    depth_.at<float>(rq, cq) = z;
                }
            }
        }
    }
}

void OdmOrthoPhoto::getBarycentricCoordiantes(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3, float x, float y, float &l1, float &l2, float &l3) const
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
