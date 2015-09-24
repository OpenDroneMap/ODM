#include "OdmTexturing.hpp"

OdmTexturing::OdmTexturing() : log_(false)
{
    logFilePath_ = "odm_texturing_log.txt";

    bundleResizedTo_ = 1200.0;
    textureWithSize_ = 2000.0;
    textureResolution_ = 4096.0;
    nrTextures_ = 0;
    padding_ = 15.0;

    mesh_ = pcl::TextureMeshPtr(new pcl::TextureMesh);
    patches_ = std::vector<Patch>(0);
    tTIA_ = std::vector<int>(0);

}

OdmTexturing::~OdmTexturing()
{

}

int OdmTexturing::run(int argc, char **argv)
{
    if (argc <= 1)
    {
        printHelp();
        return EXIT_SUCCESS;
    }

    try
    {
        parseArguments(argc, argv);
        loadMesh();
        loadCameras();
        triangleToImageAssignment();
        calculatePatches();
        sortPatches();
        createTextures();
        writeObjFile();
    }
    catch (const OdmTexturingException& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in OdmTexturing:\n";
        log_ << e.what() << "\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in OdmTexturing:\n";
        log_ << e.what() << "\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (...)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Unknown error in OdmTexturing:\n";
        log_.printToFile(logFilePath_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }

    log_.printToFile(logFilePath_);
    return EXIT_SUCCESS;
}

void OdmTexturing::parseArguments(int argc, char** argv)
{
    for(int argIndex = 1; argIndex < argc; ++argIndex)
    {
        // The argument to be parsed
        std::string argument = std::string(argv[argIndex]);
        if (argument == "-help")
        {
            printHelp();
        }
        else if (argument == "-verbose")
        {
            log_.setIsPrintingInCout(true);
        }
        else if (argument == "-bundleFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            bundlePath_ = std::string(argv[argIndex]);
            std::ifstream testFile(bundlePath_.c_str(), std::ios_base::binary);
            if (!testFile.is_open())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value (file not accessible).");
            }
            log_ << "Bundle path was set to: " << bundlePath_ << "\n";
        }
        else if (argument == "-imagesPath")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> imagesPath_;
            if (ss.bad())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "Images path was set to: " << imagesPath_ << "\n";
        }
        else if (argument == "-imagesListPath")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            imagesListPath_ = std::string(argv[argIndex]);
            std::ifstream testFile(imagesListPath_.c_str(), std::ios_base::binary);
            if (!testFile.is_open())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value (file not accessible).");
            }
            log_ << "Images list path was set to: " << imagesListPath_ << "\n";
        }
        else if (argument == "-inputModelPath")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            inputModelPath_ = std::string(argv[argIndex]);
            std::ifstream testFile(inputModelPath_.c_str(), std::ios_base::binary);
            if (!testFile.is_open())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value (file not accessible).");

            }
            log_ << "Input model path was set to: " << inputModelPath_ << "\n";
        }
        else if (argument == "-outputFolder")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> outputFolder_;
            if (ss.bad())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "Output folder path was set to: " << outputFolder_ << "\n";
        }
        else if (argument == "-logFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            logFilePath_ = std::string(argv[argIndex]);
            std::ofstream testFile(logFilePath_.c_str());
            if (!testFile.is_open())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value.");
            }
            log_ << "Log file path was set to: " << logFilePath_ << "\n";
        }
        else if (argument == "-textureResolution")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> textureResolution_;
            if (ss.bad())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "The texture resolution was set to: " << textureResolution_ << "\n";
        }
        else if (argument == "-textureWithSize")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> textureWithSize_;
            if (ss.bad())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "The resolution to texture with was set to: " << textureWithSize_ << "\n";
        }
        else if (argument == "-bundleResizedTo")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw OdmTexturingException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> bundleResizedTo_;
            if (ss.bad())
            {
                throw OdmTexturingException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "The resized resolution used in bundler was set to: " << bundleResizedTo_ << "\n";
        }
        else
        {
            printHelp();
            throw OdmTexturingException("Unrecognized argument '" + argument + "'.");
        }
    }

    if (textureWithSize_ > textureResolution_)
    {
        textureWithSize_ = textureResolution_;
        log_ << "textureWithSize parameter was set to a lower value since it can not be greater than the texture resolution.\n";
    }

}

void OdmTexturing::loadMesh()
{
    // Read model from ply-file
    pcl::PolygonMeshPtr plyMeshPtr(new pcl::PolygonMesh);
    if (pcl::io::loadPLYFile(inputModelPath_, *plyMeshPtr.get()) == -1)
    {
        throw OdmTexturingException("Error when reading model from:\n" + inputModelPath_ + "\n");
    }
    else
    {
        log_ << "Successfully loaded " << plyMeshPtr->polygons.size() << " polygons from file.\n";
    }

    // Transfer data from ply file to TextureMesh
    mesh_->cloud = plyMeshPtr->cloud;
    std::vector<pcl::Vertices> polygons;

    // Push faces from ply-mesh into TextureMesh
    polygons.resize(plyMeshPtr->polygons.size());
    for (size_t i = 0; i < plyMeshPtr->polygons.size(); ++i)
    {
        polygons[i] = plyMeshPtr->polygons[i];
    }
    mesh_->tex_polygons.push_back(polygons);

}

void OdmTexturing::loadCameras()
{
    std::ifstream bundleFile, imageListFile;
    bundleFile.open(bundlePath_.c_str(), std::ios_base::binary);
    imageListFile.open(imagesListPath_.c_str(), std::ios_base::binary);

    // Check if file is open
    if(!bundleFile.is_open())
    {
        throw OdmTexturingException("Error when reading the bundle file.");
    }
    else
    {
        log_ << "Successfully read the bundle file.\n";
    }

    if (!imageListFile.is_open())
    {
        throw OdmTexturingException("Error when reading the image list file.");
    }
    else
    {
        log_ << "Successfully read the image list file.\n";
    }

    // A temporary storage for a line from the file.
    std::string dummyLine = "";

    std::getline(bundleFile, dummyLine);

    int nrCameras= 0;
    bundleFile >> nrCameras;
    bundleFile >> dummyLine;
    for (int i = 0; i < nrCameras;++i)
    {
        double val;
        pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
        Eigen::Affine3f transform;
        bundleFile >> val; //Read focal length from bundle file
        cam.focal_length = val;
        bundleFile >> val; //Read k1 from bundle file
        bundleFile >> val; //Read k2 from bundle file

        bundleFile >> val; transform(0,0) = val; // Read rotation (0,0) from bundle file
        bundleFile >> val; transform(0,1) = val; // Read rotation (0,1) from bundle file
        bundleFile >> val; transform(0,2) = val; // Read rotation (0,2) from bundle file

        bundleFile >> val; transform(1,0) = val; // Read rotation (1,0) from bundle file
        bundleFile >> val; transform(1,1) = val; // Read rotation (1,1) from bundle file
        bundleFile >> val; transform(1,2) = val; // Read rotation (1,2) from bundle file

        bundleFile >> val; transform(2,0) = val; // Read rotation (2,0) from bundle file
        bundleFile >> val; transform(2,1) = val; // Read rotation (2,1) from bundle file
        bundleFile >> val; transform(2,2) = val; // Read rotation (2,2) from bundle file

        bundleFile >> val; transform(0,3) = val; // Read translation (0,3) from bundle file
        bundleFile >> val; transform(1,3) = val; // Read translation (1,3) from bundle file
        bundleFile >> val; transform(2,3) = val; // Read translation (2,3) from bundle file

        transform(3,0) = 0.0;
        transform(3,1) = 0.0;
        transform(3,2) = 0.0;
        transform(3,3) = 1.0;

        transform = transform.inverse();

        // Column negation
        transform(0,2) = -1.0*transform(0,2);
        transform(1,2) = -1.0*transform(1,2);
        transform(2,2) = -1.0*transform(2,2);

        transform(0,1) = -1.0*transform(0,1);
        transform(1,1) = -1.0*transform(1,1);
        transform(2,1) = -1.0*transform(2,1);

        // Set values from bundle to current camera
        cam.pose = transform;

        std::getline(imageListFile, dummyLine);
        size_t firstWhitespace = dummyLine.find_first_of(" ");

        if (firstWhitespace != std::string::npos)
        {
            cam.texture_file = imagesPath_ + "/" + dummyLine.substr(2,firstWhitespace-2);
        }
        else
        {
            cam.texture_file = imagesPath_ + "/" + dummyLine.substr(2);
        }

        // Read image to get full resolution size
        cv::Mat image = cv::imread(cam.texture_file);

        if (image.empty())
        {
            throw OdmTexturingException("Failed to read image:\n'" + cam.texture_file + "'\n");
        }

        double imageWidth = static_cast<double>(image.cols);
        double textureWithWidth = static_cast<double>(textureWithSize_);

        // Calculate scale factor to texture with textureWithSize
        double factor = textureWithWidth/imageWidth;
        if (factor > 1.0f)
        {
            factor = 1.0f;
        }

        // Update camera size and focal length
        cam.height = static_cast<int>(std::floor(factor*(static_cast<double>(image.rows))));
        cam.width = static_cast<int>(std::floor(factor*(static_cast<double>(image.cols))));
        cam.focal_length *= static_cast<double>(cam.width)/bundleResizedTo_;

        // Add camera
        cameras_.push_back(cam);

    }

}

void OdmTexturing::triangleToImageAssignment()
{
    // Resize the triangleToImageAssigmnent vector to match the number of faces in the mesh
    tTIA_.resize(mesh_->tex_polygons[0].size());

    // Set all values in the triangleToImageAssignment vector to a default value (-1) if there are no optimal camera
    for (size_t i = 0; i < tTIA_.size(); ++i)
    {
        tTIA_[i] = -1;
    }

    // Vector containing information if the face has been given an optimal camera or not
    std::vector<bool> hasOptimalCamera = std::vector<bool>(mesh_->tex_polygons[0].size());

    // Set default value that no face has an optimal camera
    for (size_t faceIndex = 0; faceIndex < hasOptimalCamera.size(); ++faceIndex)
    {
        hasOptimalCamera[faceIndex] = false;
    }

    // Convert vertices to pcl::PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh_->cloud, *meshCloud);

    // Create dummy point and UV-index for vertices not visible in any cameras
    pcl::PointXY nanPoint;
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    pcl::texture_mapping::UvIndex uvNull;
    uvNull.idx_cloud = -1;
    uvNull.idx_face = -1;

    for (size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
    {
        // Move vertices in mesh into the camera coordinate system
        pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*meshCloud, *cameraCloud, cameras_[cameraIndex].pose.inverse());

        // Cloud to contain points projected into current camera
        pcl::PointCloud<pcl::PointXY>::Ptr projections (new pcl::PointCloud<pcl::PointXY>);

        // Vector containing information if the polygon is visible in current camera
        std::vector<bool> visibility;
        visibility.resize(mesh_->tex_polygons[0].size());

        // Vector for remembering the correspondence between uv-coordinates and faces
        std::vector<pcl::texture_mapping::UvIndex> indexUvToPoints;

        // Count the number of vertices inside the camera frustum
        int countInsideFrustum = 0;

        // Frustum culling for all faces
        for (size_t faceIndex = 0; faceIndex < mesh_->tex_polygons[0].size(); ++faceIndex)
        {
            // Variables for the face vertices as projections in the camera plane
            pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;

            // If the face is inside the camera frustum
            if (isFaceProjected(cameras_[cameraIndex],
                                cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[0]],
                                cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[1]],
                                cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[2]],
                                pixelPos0, pixelPos1, pixelPos2))
            {
                // Add pixel positions in camera to projections
                projections->points.push_back((pixelPos0));
                projections->points.push_back((pixelPos1));
                projections->points.push_back((pixelPos2));

                // Remember corresponding face
                pcl::texture_mapping::UvIndex u1, u2, u3;
                u1.idx_cloud = mesh_->tex_polygons[0][faceIndex].vertices[0];
                u2.idx_cloud = mesh_->tex_polygons[0][faceIndex].vertices[1];
                u3.idx_cloud = mesh_->tex_polygons[0][faceIndex].vertices[2];
                u1.idx_face = faceIndex; u2.idx_face = faceIndex; u3.idx_face = faceIndex;
                indexUvToPoints.push_back(u1);
                indexUvToPoints.push_back(u2);
                indexUvToPoints.push_back(u3);

                // Update visibility vector
                visibility[faceIndex] = true;

                // Update count
                ++countInsideFrustum;
            }
            else
            {
                // If not visible set nanPoint and uvNull
                projections->points.push_back(nanPoint);
                projections->points.push_back(nanPoint);
                projections->points.push_back(nanPoint);
                indexUvToPoints.push_back(uvNull);
                indexUvToPoints.push_back(uvNull);
                indexUvToPoints.push_back(uvNull);

                // Update visibility vector
                visibility[faceIndex] = false;
            }


        }

        // If any faces are visible in the current camera perform occlusion culling
        if (countInsideFrustum > 0)
        {
            // Set up acceleration structure
            pcl::KdTreeFLANN<pcl::PointXY> kdTree;
            kdTree.setInputCloud(projections);

            // Loop through all faces and perform occlusion culling for faces inside frustum
            for (size_t faceIndex = 0; faceIndex < mesh_->tex_polygons[0].size(); ++faceIndex)
            {
                if (visibility[faceIndex])
                {
                    // Vectors to store output from radiusSearch in acceleration structure
                    std::vector<int> neighbors; std::vector<float> neighborsSquaredDistance;

                    // Variables for the vertices in face as projections in the camera plane
                    pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;

                    if (isFaceProjected(cameras_[cameraIndex],
                                        cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[0]],
                                        cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[1]],
                                        cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[2]],
                                        pixelPos0, pixelPos1, pixelPos2))
                    {
                        // Variables for a radius circumscribing the polygon in the camera plane and the center of the polygon
                        double radius; pcl::PointXY center;

                        // Get values for radius and center
                        getTriangleCircumscribedCircleCentroid(pixelPos0, pixelPos1, pixelPos2, center, radius);

                        // Perform radius search in the acceleration structure
                        int radiusSearch = kdTree.radiusSearch(center, radius, neighbors, neighborsSquaredDistance);

                        // If other projections are found inside the radius
                        if (radiusSearch > 0)
                        {
                            // Extract distances for all vertices for face to camera
                            double d0 = cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[0]].z;
                            double d1 = cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[1]].z;
                            double d2 = cameraCloud->points[mesh_->tex_polygons[0][faceIndex].vertices[2]].z;

                            // Calculate largest distance and store in distance variable
                            double distance = std::max(d0, std::max(d1,d2));

                            // Compare distance to all neighbors inside radius
                            for (size_t i = 0; i < neighbors.size(); ++i)
                            {
                                // Distance variable from neighbor to camera
                                double neighborDistance = cameraCloud->points[indexUvToPoints[neighbors[i]].idx_cloud].z;

                                // If the neighbor has a greater distance to the camera and is inside face polygon set it as not visible
                                if (distance < neighborDistance)
                                {
                                      if (checkPointInsideTriangle(pixelPos0, pixelPos1, pixelPos2, projections->points[neighbors[i]]))
                                      {
                                          // Update visibility for neighbors
                                          visibility[indexUvToPoints[neighbors[i]].idx_face] = false;
                                      }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Number of polygons that add current camera as the optimal camera
        int count = 0;

        // Update optimal cameras for faces visible in current camera
        for (size_t faceIndex = 0; faceIndex < visibility.size();++faceIndex)
        {
            if (visibility[faceIndex])
            {
                hasOptimalCamera[faceIndex] = true;
                tTIA_[faceIndex] = cameraIndex;
                ++count;
            }
        }

    }

}

void OdmTexturing::calculatePatches()
{
    // Convert vertices to pcl::PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh_->cloud, *meshCloud);

    // Reserve size for patches_
    patches_.reserve(tTIA_.size());

    // Vector containing vector with indicies to faces visible in corresponding camera index
    std::vector<std::vector<int> > optFaceCameraVector = std::vector<std::vector<int> >(cameras_.size());

    // Counter variables for visible and occluded faces
    int countVis = 0;
    int countOcc = 0;

    Patch nonVisibleFaces;
    nonVisibleFaces.optimalCameraIndex_ = -1;
    nonVisibleFaces.materialIndex_ = -1;
    nonVisibleFaces.placed_ = true;

    // Setup vector containing vectors with all faces correspondning to camera according to triangleToImageAssignment vector
    for (size_t i = 0; i < tTIA_.size(); ++i)
    {
        if (tTIA_[i] > -1)
        {
            // If face has an optimal camera add to optFaceCameraVector and update counter for visible faces
            countVis++;
            optFaceCameraVector[tTIA_[i]].push_back(i);
        }
        else
        {
            // Add non visible face to patch nonVisibleFaces
            nonVisibleFaces.faces_.push_back(i);

            // Update counter for occluded faces
            countOcc++;
        }
    }

    log_ << "Visible faces: "<<countVis<<"\n";
    log_ << "Occluded faces: "<<countOcc<<"\n";

    // Loop through all cameras
    for (size_t cameraIndex = 0; cameraIndex < cameras_.size(); ++cameraIndex)
    {
        // Transform mesh into camera coordinate system
        pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*meshCloud, *cameraCloud, cameras_[cameraIndex].pose.inverse());

        // While faces visible in camera remains to be assigned to a patch
        while(0 < optFaceCameraVector[cameraIndex].size())
        {
            // Create current patch
            Patch patch;

            // Vector containing faces to check connectivity with current patch
            std::vector<size_t> addedFaces = std::vector<size_t>(0);

            // Add last face in optFaceCameraVector to faces to check connectivity and add it to the current patch
            addedFaces.push_back(optFaceCameraVector[cameraIndex].back());

            // Add first face to patch
            patch.faces_.push_back(optFaceCameraVector[cameraIndex].back());

            // Remove face from optFaceCameraVector
            optFaceCameraVector[cameraIndex].pop_back();

            // Declare uv-coordinates for face
            pcl::PointXY uvCoord1; pcl::PointXY uvCoord2; pcl::PointXY uvCoord3;

            // Calculate uv-coordinates for face in camera
            if (isFaceProjected(cameras_[cameraIndex],
                            cameraCloud->points[mesh_->tex_polygons[0][addedFaces.back()].vertices[0]],
                            cameraCloud->points[mesh_->tex_polygons[0][addedFaces.back()].vertices[1]],
                            cameraCloud->points[mesh_->tex_polygons[0][addedFaces.back()].vertices[2]],
                            uvCoord1, uvCoord2, uvCoord3))
            {
                // Set minimum and maximum uv-coordinate value for patch
                patch.minu_ = std::min(uvCoord1.x, std::min(uvCoord2.x, uvCoord3.x));
                patch.minv_ = std::min(uvCoord1.y, std::min(uvCoord2.y, uvCoord3.y));
                patch.maxu_ = std::max(uvCoord1.x, std::max(uvCoord2.x, uvCoord3.x));
                patch.maxv_ = std::max(uvCoord1.y, std::max(uvCoord2.y, uvCoord3.y));

                while(0 < addedFaces.size())
                {
                    // Set face to check neighbors
                    size_t patchFaceIndex = addedFaces.back();

                    // Remove patchFaceIndex from addedFaces
                    addedFaces.pop_back();

                    // Check against all remaining faces with the same optimal camera
                    for (size_t i = 0; i < optFaceCameraVector[cameraIndex].size(); ++i)
                    {
                        size_t modelFaceIndex = optFaceCameraVector[cameraIndex][i];

                        // Don't check against self
                        if (modelFaceIndex != patchFaceIndex)
                        {
                            // Store indices for vertices of both faces
                            size_t face0v0 = mesh_->tex_polygons[0][modelFaceIndex].vertices[0];
                            size_t face0v1 = mesh_->tex_polygons[0][modelFaceIndex].vertices[1];
                            size_t face0v2 = mesh_->tex_polygons[0][modelFaceIndex].vertices[2];
                            size_t face1v0 = mesh_->tex_polygons[0][patchFaceIndex].vertices[0];
                            size_t face1v1 = mesh_->tex_polygons[0][patchFaceIndex].vertices[1];
                            size_t face1v2 = mesh_->tex_polygons[0][patchFaceIndex].vertices[2];

                            // Count the number of shared vertices
                            size_t nShared = 0;
                            nShared += (face0v0 == face1v0 ? 1 : 0) + (face0v0 == face1v1 ? 1 : 0) + (face0v0 == face1v2 ? 1 : 0);
                            nShared += (face0v1 == face1v0 ? 1 : 0) + (face0v1 == face1v1 ? 1 : 0) + (face0v1 == face1v2 ? 1 : 0);
                            nShared += (face0v2 == face1v0 ? 1 : 0) + (face0v2 == face1v1 ? 1 : 0) + (face0v2 == face1v2 ? 1 : 0);

                            // If sharing a vertex
                            if (nShared > 0)
                            {
                                // Declare uv-coordinates for face
                                pcl::PointXY uv1; pcl::PointXY uv2; pcl::PointXY uv3;

                                // Calculate uv-coordinates for face in camera
                                isFaceProjected(cameras_[cameraIndex],
                                                cameraCloud->points[mesh_->tex_polygons[0][modelFaceIndex].vertices[0]],
                                                cameraCloud->points[mesh_->tex_polygons[0][modelFaceIndex].vertices[1]],
                                                cameraCloud->points[mesh_->tex_polygons[0][modelFaceIndex].vertices[2]],
                                                uv1, uv2, uv3);

                                // Update minimum and maximum uv-coordinate value for patch
                                patch.minu_ = std::min(patch.minu_, std::min(uv1.x, std::min(uv2.x, uv3.x)));
                                patch.minv_ = std::min(patch.minv_, std::min(uv1.y, std::min(uv2.y, uv3.y)));
                                patch.maxu_ = std::max(patch.maxu_, std::max(uv1.x, std::max(uv2.x, uv3.x)));
                                patch.maxv_ = std::max(patch.maxv_, std::max(uv1.y, std::max(uv2.y, uv3.y)));

                                // Add modelFaceIndex to patch
                                patch.faces_.push_back(modelFaceIndex);

                                // Add modelFaceIndex from faces to check for neighbors with same optimal camera
                                addedFaces.push_back(modelFaceIndex);

                                // Remove modelFaceIndex from optFaceCameraVector to exclude it from comming iterations
                                optFaceCameraVector[cameraIndex].erase(optFaceCameraVector[cameraIndex].begin() + i);
                            }
                        }
                    }
                }
            }

            // Set optimal camera for patch
            patch.optimalCameraIndex_ = static_cast<int>(cameraIndex);

            // Add patch to patches_ vector
            patches_.push_back(patch);
        }
    }
    patches_.push_back(nonVisibleFaces);
}

Coords OdmTexturing::recursiveFindCoords(Node &n, float w, float h)
{
    // Coordinates to return and place patch
    Coords c;

    if (NULL != n.lft_)
    {
        c = recursiveFindCoords(*(n.lft_), w, h);
        if (c.success_)
        {
            return c;
        }
        else
        {
            return recursiveFindCoords(*(n.rgt_), w, h);
        }
    }
    else
    {
        // If the patch is to large or occupied return success false for coord
        if (n.used_ || w > n.width_ || h > n.height_)
        {
            c.success_ = false;
            return c;
        }

        // If the patch matches perfectly, store it
        if (w == n.width_ && h == n.height_)
        {
            n.used_ = true;
            c.r_ = n.r_;
            c.c_ = n.c_;
            c.success_ = true;

            return c;
        }

        // Initialize children for node
        n.lft_ = new Node(n);
        n.rgt_ = new Node(n);

        n.rgt_->used_ = false;
        n.lft_->used_ = false;
        n.rgt_->rgt_ = NULL;
        n.rgt_->lft_ = NULL;
        n.lft_->rgt_ = NULL;
        n.lft_->lft_ = NULL;

        // Check how to adjust free space
        if (n.width_ - w > n.height_ - h)
        {
            n.lft_->width_ = w;
            n.rgt_->c_ = n.c_ + w;
            n.rgt_->width_ = n.width_ - w;
        }
        else
        {
            n.lft_->height_ = h;
            n.rgt_->r_ = n.r_ + h;
            n.rgt_->height_ = n.height_ - h;
        }

        return recursiveFindCoords(*(n.lft_), w, h);
    }
}

void OdmTexturing::sortPatches()
{
    // Bool to set true when done
    bool done = false;

    // Material index
    int materialIndex = 0;

    // Number of patches left from last loop
    size_t countLeftLastIteration = 0;

    while(!done)
    {
        // Create container for current material
        Node root;
        root.width_ = textureResolution_;
        root.height_ = textureResolution_;

        // Set done to true
        done = true;

        // Number of patches that did not fit in current material
        size_t countNotPlacedPatches = 0;

        // Number of patches placed in current material
        size_t placed = 0;

        for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
        {
            if(!patches_[patchIndex].placed_)
            {
                // Calculate dimensions of the patch
                float w = patches_[patchIndex].maxu_ - patches_[patchIndex].minu_ + 2*padding_;
                float h = patches_[patchIndex].maxv_ - patches_[patchIndex].minv_ + 2*padding_;

                // Try to place patch in root container for this material
                if ( w > 0.0 && h > 0.0)
                {
                    patches_[patchIndex].c_ = recursiveFindCoords(root, w, h);
                }

                if (!patches_[patchIndex].c_.success_)
                {
                    ++countNotPlacedPatches;
                    done = false;
                }
                else
                {
                    // Set patch material as current material
                    patches_[patchIndex].materialIndex_ = materialIndex;

                    // Set patch as placed
                    patches_[patchIndex].placed_ = true;

                    // Update number of patches placed in current material
                    placed++;

                    // Update patch with padding_
                    //patches_[patchIndex].c_.c_ += padding_;
                    //patches_[patchIndex].c_.r_ += padding_;
                    patches_[patchIndex].minu_ -= padding_;
                    patches_[patchIndex].minv_ -= padding_;
                    patches_[patchIndex].maxu_ = std::min((patches_[patchIndex].maxu_ + padding_), textureResolution_);
                    patches_[patchIndex].maxv_ = std::min((patches_[patchIndex].maxv_ + padding_), textureResolution_);
                }
            }
        }

        // Update material index
        ++materialIndex;

        if (countLeftLastIteration == countNotPlacedPatches && countNotPlacedPatches != 0)
        {
            done = true;
        }
        countLeftLastIteration = countNotPlacedPatches;
    }

    // Set number of textures
    nrTextures_ =  materialIndex;

    log_ << "Faces sorted into " << nrTextures_ << " textures.\n";
}

void OdmTexturing::createTextures()
{
    // Convert vertices to pcl::PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh_->cloud, *meshCloud);

    // Container for faces according to submesh. Used to replace faces in mesh_.
    std::vector<std::vector<pcl::Vertices> > faceVector = std::vector<std::vector<pcl::Vertices> >(nrTextures_ + 1);

    // Container for texture coordinates according to submesh. Used to replace texture coordinates in mesh_.
    std::vector<std::vector<Eigen::Vector2f> > textureCoordinatesVector = std::vector<std::vector<Eigen::Vector2f> >(nrTextures_ + 1);

    // Container for materials according to submesh. Used to replace materials in mesh_.
    std::vector<pcl::TexMaterial> materialVector = std::vector<pcl::TexMaterial>(nrTextures_ + 1);

    // Setup model according to patches placement
    for (int textureIndex = 0; textureIndex < nrTextures_; ++textureIndex)
    {
        for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
        {
            // If patch is placed in current mesh add all containing faces to that submesh
            if (patches_[patchIndex].materialIndex_ == textureIndex)
            {
                // Transform mesh into camera
                pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud (*meshCloud, *cameraCloud, cameras_[patches_[patchIndex].optimalCameraIndex_].pose.inverse());

                // Loop through all faces in patch
                for (size_t faceIndex = 0; faceIndex < patches_[patchIndex].faces_.size(); ++faceIndex)
                {
                    // Setup global face index in mesh_
                    size_t globalFaceIndex = patches_[patchIndex].faces_[faceIndex];

                    // Add current face to current submesh
                    faceVector[textureIndex].push_back(mesh_->tex_polygons[0][globalFaceIndex]);

                    // Pixel positions
                    pcl::PointXY pixelPos0; pcl::PointXY pixelPos1; pcl::PointXY pixelPos2;

                    // Get pixel positions in corresponding camera for the vertices of the face
                    getPixelCoordinates(cameraCloud->points[mesh_->tex_polygons[0][globalFaceIndex].vertices[0]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos0);
                    getPixelCoordinates(cameraCloud->points[mesh_->tex_polygons[0][globalFaceIndex].vertices[1]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos1);
                    getPixelCoordinates(cameraCloud->points[mesh_->tex_polygons[0][globalFaceIndex].vertices[2]], cameras_[patches_[patchIndex].optimalCameraIndex_], pixelPos2);

                    // Shorthands for patch variables
                    float c = patches_[patchIndex].c_.c_ + padding_;
                    float r = patches_[patchIndex].c_.r_ + padding_;
                    float minu = patches_[patchIndex].minu_ + padding_;
                    float minv = patches_[patchIndex].minv_ + padding_;

                    // Declare uv coordinates
                    Eigen::Vector2f uv1, uv2, uv3;

                    // Set uv coordinates according to patch
                    uv1(0) = (pixelPos0.x - minu + c)/textureResolution_;
                    uv1(1) = 1.0f - (pixelPos0.y - minv + r)/textureResolution_;

                    uv2(0) = (pixelPos1.x - minu + c)/textureResolution_;
                    uv2(1) = 1.0f - (pixelPos1.y - minv + r)/textureResolution_;

                    uv3(0) = (pixelPos2.x - minu + c)/textureResolution_;
                    uv3(1) = 1.0f - (pixelPos2.y - minv + r)/textureResolution_;

                    // Add uv coordinates to submesh
                    textureCoordinatesVector[textureIndex].push_back(uv1);
                    textureCoordinatesVector[textureIndex].push_back(uv2);
                    textureCoordinatesVector[textureIndex].push_back(uv3);
                }
            }
        }

        // Declare material and setup default values
        pcl::TexMaterial meshMaterial;
        meshMaterial.tex_Ka.r = 0.0f; meshMaterial.tex_Ka.g = 0.0f; meshMaterial.tex_Ka.b = 0.0f;
        meshMaterial.tex_Kd.r = 0.0f; meshMaterial.tex_Kd.g = 0.0f; meshMaterial.tex_Kd.b = 0.0f;
        meshMaterial.tex_Ks.r = 0.0f; meshMaterial.tex_Ks.g = 0.0f; meshMaterial.tex_Ks.b = 0.0f;
        meshMaterial.tex_d = 1.0f; meshMaterial.tex_Ns = 200.0f; meshMaterial.tex_illum = 2;
        std::stringstream tex_name;
        tex_name << "texture_" << textureIndex;
        tex_name >> meshMaterial.tex_name;
        meshMaterial.tex_file = meshMaterial.tex_name + ".jpg";
        materialVector[textureIndex] = meshMaterial;
    }

    // Add non visible patches to submesh
    for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
    {
        // If the patch does not have an optimal camera
        if (patches_[patchIndex].optimalCameraIndex_ == -1)
        {
            // Add all faces and set uv coordinates
            for (size_t faceIndex = 0; faceIndex < patches_[patchIndex].faces_.size(); ++faceIndex)
            {
                // Setup global face index in mesh_
                size_t globalFaceIndex = patches_[patchIndex].faces_[faceIndex];

                // Add current face to current submesh
                faceVector[nrTextures_].push_back(mesh_->tex_polygons[0][globalFaceIndex]);

                // Declare uv coordinates
                Eigen::Vector2f uv1, uv2, uv3;

                // Set uv coordinates according to patch
                uv1(0) = 0.25f;//(pixelPos0.x - minu + c)/textureResolution_;
                uv1(1) = 0.25f;//1.0f - (pixelPos0.y - minv + r)/textureResolution_;

                uv2(0) = 0.25f;//(pixelPos1.x - minu + c)/textureResolution_;
                uv2(1) = 0.75f;//1.0f - (pixelPos1.y - minv + r)/textureResolution_;

                uv3(0) = 0.75f;//(pixelPos2.x - minu + c)/textureResolution_;
                uv3(1) = 0.75f;//1.0f - (pixelPos2.y - minv + r)/textureResolution_;

                // Add uv coordinates to submesh
                textureCoordinatesVector[nrTextures_].push_back(uv1);
                textureCoordinatesVector[nrTextures_].push_back(uv2);
                textureCoordinatesVector[nrTextures_].push_back(uv3);
            }
        }
    }

    // Declare material and setup default values for nonVisibileFaces submesh
    pcl::TexMaterial meshMaterial;
    meshMaterial.tex_Ka.r = 0.0f; meshMaterial.tex_Ka.g = 0.0f; meshMaterial.tex_Ka.b = 0.0f;
    meshMaterial.tex_Kd.r = 0.0f; meshMaterial.tex_Kd.g = 0.0f; meshMaterial.tex_Kd.b = 0.0f;
    meshMaterial.tex_Ks.r = 0.0f; meshMaterial.tex_Ks.g = 0.0f; meshMaterial.tex_Ks.b = 0.0f;
    meshMaterial.tex_d = 1.0f; meshMaterial.tex_Ns = 200.0f; meshMaterial.tex_illum = 2;
    std::stringstream tex_name;
    tex_name << "non_visible_faces_texture";
    tex_name >> meshMaterial.tex_name;
    meshMaterial.tex_file = meshMaterial.tex_name + ".jpg";
    materialVector[nrTextures_] = meshMaterial;

    // Replace polygons, texture coordinates and materials in mesh_
    mesh_->tex_polygons = faceVector;
    mesh_->tex_coordinates = textureCoordinatesVector;
    mesh_->tex_materials = materialVector;

    // Containers for image and the resized image used for texturing
    cv::Mat image;
    cv::Mat resizedImage;

    for (int textureIndex = 0; textureIndex < nrTextures_; ++textureIndex)
    {
        // Current texture for corresponding material
        cv::Mat texture = cv::Mat::zeros(textureResolution_, textureResolution_, CV_8UC3);

        for (int cameraIndex = 0; cameraIndex < static_cast<int>(cameras_.size()); ++cameraIndex)
        {
            // Load image for current camera
            image = cv::imread(cameras_[cameraIndex].texture_file,1);

            // Calculate the resize factor to texturize with textureWithSize_
            double resizeFactor = textureWithSize_ / static_cast<double>(image.cols);

            if (resizeFactor > 1.0f)
            {
                resizeFactor = 1.0f;
            }

            // Resize image to the resolution used to texture with
            cv::resize(image, resizedImage, cv::Size(), resizeFactor, resizeFactor, CV_INTER_AREA);

            // Loop through all patches
            for (size_t patchIndex = 0; patchIndex < patches_.size(); ++patchIndex)
            {
                // If the patch has the current camera as optimal camera
                if (patches_[patchIndex].materialIndex_ == textureIndex && patches_[patchIndex].optimalCameraIndex_ == cameraIndex)
                {
                    // Pixel coordinates to extract image information from
                    int extractX = static_cast<int>(floor(patches_[patchIndex].minu_));// + padding_);
                    int extractY = static_cast<int>(floor(patches_[patchIndex].minv_));// + padding_);

                    // Pixel coordinates to insert the image information to
                    int insertX = static_cast<int>(floor(patches_[patchIndex].c_.c_));
                    int insertY = static_cast<int>(floor(patches_[patchIndex].c_.r_));

                    // The size of the image information to use
                    int width = static_cast<int>(floor(patches_[patchIndex].maxu_)) - extractX - 1;
                    int height = static_cast<int>(floor(patches_[patchIndex].maxv_)) - extractY - 1;

                    // Get image information and add to texture
                    cv::Mat src =  resizedImage(cv::Rect(extractX, extractY, width, height));
                    cv::Mat dst = texture(cv::Rect(insertX, insertY, width, height));
                    src.copyTo(dst);
                }
            }
        }
        log_ <<"Saved " << mesh_->tex_materials[textureIndex].tex_file <<" to file.\n";
        cv::imwrite(outputFolder_ + mesh_->tex_materials[textureIndex].tex_file, texture);
    }

    // Create nonVisibleFaces texture and save to file
    cv::Mat nonVisibleFacesTexture = cv::Mat::zeros(50, 50, CV_8UC3) + cv::Scalar(255,255,255);
    log_ <<"Saved " << mesh_->tex_materials[nrTextures_].tex_file <<" to file.\n";
    cv::imwrite(outputFolder_ + mesh_->tex_materials[nrTextures_].tex_file, nonVisibleFacesTexture);
}

void OdmTexturing::writeObjFile()
{
    if (saveOBJFile(outputFolder_ + "odm_textured_model.obj", *mesh_.get(), 7) == 0)
    {
        log_ <<"odm_textured_model.obj successfully saved.\n";
    }
    else
    {
        log_ << "Failed to save model.\n";
    }
}

void OdmTexturing::printHelp()
{
    log_.setIsPrintingInCout(true);

    log_ << "Purpose:\n";
    log_ << "Create a textured mesh from pmvs output and a corresponding ply-mesh from OpenDroneMap-meshing.\n";

    log_ << "Usage:\n";
    log_ << "The program requires paths to a bundler.out file, a ply model file, an image list file, an image folder path, an output path and the resized resolution used in the bundler step.\n";
    log_ << "All other input parameters are optional.\n\n";

    log_ << "The following flags are available:\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output.\n\n";

    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configurable:\n";
    log_ << "\"-bundleFile <path>\" (mandatory)\n";
    log_ << "Path to the bundle.out file from pmvs.\n";

    log_ << "\"-imagesListPath <path>\" (mandatory)\n";
    log_ << "Path to the list containing the image names used in the bundle.out file.\n";

    log_ << "\"-imagesPath <path>\" (mandatory)\n";
    log_ << "Path to the folder containing full resolution images.\n";

    log_ << "\"-inputModelPath <path>\" (mandatory)\n";
    log_ << "Path to the ply model to texture.\n";

    log_ << "\"-outputFolder <path>\" (mandatory)\n";
    log_ << "Path to store the textured model. The folder must exist.\n";

    log_ << "\"-logFile <path>\" (optional)\n";
    log_ << "Path to save the log file.\n";

    log_ << "\"-bundleResizedTo <integer>\" (mandatory)\n";
    log_ << "The resized resolution used in bundler.\n";

    log_ << "\"-textureResolution_ <integer>\" (optional, default: 4096)\n";
    log_ << "The resolution of the output textures. Must be greater than textureWithSize.\n";

    log_ << "\"-textureWithSize <integer>\" (optional, default: 1200)\n";
    log_ << "The resolution to rescale the images performing the texturing.\n";

    log_.setIsPrintingInCout(false);

}
