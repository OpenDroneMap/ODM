#include "Odm25dMeshing.hpp"

int Odm25dMeshing::run(int argc, char **argv) {
	log << logFilePath << "\n";

	// If no arguments were passed, print help and return early.
	if (argc <= 1) {
		printHelp();
		return EXIT_SUCCESS;
	}

	try {

		parseArguments(argc, argv);

		loadPointCloud();

		buildMesh();

	} catch (const Odm25dMeshingException& e) {
		log.setIsPrintingInCout(true);
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	} catch (const std::exception& e) {
		log.setIsPrintingInCout(true);
		log << "Error in OdmMeshing:\n";
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	}

	log.printToFile(logFilePath);

	return EXIT_SUCCESS;
}

void Odm25dMeshing::loadPointCloud() {
	log << "Loading point cloud... ";

	try{
		std::ifstream ss(inputFile, std::ios::binary);
		if (ss.fail()) throw Odm25dMeshingException("Failed to open " + inputFile);
		PlyFile file;

		file.parse_header(ss);

		std::shared_ptr<PlyData> vertices = file.request_properties_from_element("vertex", { "x", "y", "z" });
		file.read(ss);

		const size_t numVerticesBytes = vertices->buffer.size_bytes();
		struct float3 { float x, y, z; };
		struct double3 { double x, y, z; };

		if (vertices->t == tinyply::Type::FLOAT32) {
			std::vector<float3> verts(vertices->count);
			std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
			for (float3 &v : verts){
				points->InsertNextPoint(v.x, v.y, v.z);
			}
		}else if (vertices->t == tinyply::Type::FLOAT64) {
			std::vector<double3> verts(vertices->count);
			std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
			for (double3 &v : verts){
				points->InsertNextPoint(v.x, v.y, v.z);
			}
		}else{
			throw Odm25dMeshingException("Invalid data type (only float32 and float64 are supported): " + std::to_string((int)vertices->t));
		}
	}
	catch (const std::exception & e)
	{
		throw Odm25dMeshingException("Error while loading point cloud: " + std::string(e.what()));
	}

	log << "loaded " << points->GetNumberOfPoints() << " points\n";
}

void Odm25dMeshing::buildMesh(){
	vtkThreadedImageAlgorithm::SetGlobalDefaultEnableSMP(true);

	log << "Remove outliers... ";

	vtkSmartPointer<vtkPolyData> polyPoints =
		  vtkSmartPointer<vtkPolyData>::New();
	polyPoints->SetPoints(points);

	vtkSmartPointer<vtkOctreePointLocator> locator = vtkSmartPointer<vtkOctreePointLocator>::New();

	vtkSmartPointer<vtkRadiusOutlierRemoval> radiusRemoval =
		vtkSmartPointer<vtkRadiusOutlierRemoval>::New();
	radiusRemoval->SetInputData(polyPoints);
	radiusRemoval->SetLocator(locator);
	radiusRemoval->SetRadius(20); // 20 meters
	radiusRemoval->SetNumberOfNeighbors(2);

	vtkSmartPointer<vtkStatisticalOutlierRemoval> statsRemoval =
			vtkSmartPointer<vtkStatisticalOutlierRemoval>::New();
	statsRemoval->SetInputConnection(radiusRemoval->GetOutputPort());
	statsRemoval->SetLocator(locator);
	statsRemoval->SetSampleSize(neighbors);
	statsRemoval->SetStandardDeviationFactor(1.5);
	statsRemoval->GenerateOutliersOff();
	statsRemoval->Update();

	log << (radiusRemoval->GetNumberOfPointsRemoved() + statsRemoval->GetNumberOfPointsRemoved()) << " points removed\n";
	vtkSmartPointer<vtkPoints> cleanedPoints = statsRemoval->GetOutput()->GetPoints();

	statsRemoval = nullptr;
	radiusRemoval = nullptr;
	polyPoints = nullptr;

	log << "Squash point cloud to plane... ";

	vtkSmartPointer<vtkFloatArray> elevation = vtkSmartPointer<vtkFloatArray>::New();
	elevation->SetName("elevation");
	elevation->SetNumberOfComponents(1);
	double p[2];

	for (vtkIdType i = 0; i < cleanedPoints->GetNumberOfPoints(); i++){
		cleanedPoints->GetPoint(i, p);
		elevation->InsertNextValue(p[2]);
		p[2] = 0.0;
		cleanedPoints->SetPoint(i, p);
	}

	log << "OK\n";

	vtkSmartPointer<vtkPolyData> polydataToProcess =
	  vtkSmartPointer<vtkPolyData>::New();
	polydataToProcess->SetPoints(cleanedPoints);
	polydataToProcess->GetPointData()->SetScalars(elevation);

	const float NODATA = -9999;

	double *bounds = polydataToProcess->GetBounds();

	double centerX = polydataToProcess->GetCenter()[0];
	double centerY = polydataToProcess->GetCenter()[1];
	double centerZ = polydataToProcess->GetCenter()[2];

	double extentX = bounds[1] - bounds[0];
	double extentY = bounds[3] - bounds[2];

	if (resolution == 0.0){
		resolution = (double)maxVertexCount / (sqrt(extentX * extentY) * 75.0);
		log << "Automatically set resolution to " << std::fixed << resolution << "\n";
	}

	int width = ceil(extentX * resolution);
	int height = ceil(extentY * resolution);

	log << "Plane extentX: " << extentX <<
				", extentY: " << extentY << "\n";

	double planeCenter[3];
	planeCenter[0] = centerX;
	planeCenter[1] = centerY;
	planeCenter[2] = centerZ;

	vtkSmartPointer<vtkPlaneSource> plane =
			vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetResolution(width, height);
	plane->SetOrigin(0.0, 0.0, 0.0);
	plane->SetPoint1(extentX, 0.0, 0.0);
	plane->SetPoint2(0.0, extentY, 0);
	plane->SetCenter(planeCenter);
	plane->SetNormal(0.0, 0.0, 1.0);

	vtkSmartPointer<vtkShepardKernel> shepardKernel =
				vtkSmartPointer<vtkShepardKernel>::New();
	shepardKernel->SetPowerParameter(2.0);
	shepardKernel->SetKernelFootprintToNClosest();
	shepardKernel->SetNumberOfPoints(neighbors);

	vtkSmartPointer<vtkImageData> image =
	    vtkSmartPointer<vtkImageData>::New();
	image->SetDimensions(width, height, 1);
	log << "DSM size is " << width << "x" << height << " (" << ceil(width * height * sizeof(float) * 1e-6) << " MB) \n";
	image->AllocateScalars(VTK_FLOAT, 1);

	log << "Point interpolation using shepard's kernel... ";

	vtkSmartPointer<vtkPointInterpolator> interpolator =
				vtkSmartPointer<vtkPointInterpolator>::New();
	interpolator->SetInputConnection(plane->GetOutputPort());
	interpolator->SetSourceData(polydataToProcess);
	interpolator->SetKernel(shepardKernel);
	interpolator->SetLocator(locator);
	interpolator->SetNullValue(NODATA);
	interpolator->Update();

	vtkSmartPointer<vtkPolyData> interpolatedPoly =
			interpolator->GetPolyDataOutput();

	log << "OK\nTransfering interpolation results to DSM... ";

	interpolator = nullptr;
	polydataToProcess = nullptr;
	elevation = nullptr;
	cleanedPoints = nullptr;
	plane = nullptr;
	shepardKernel = nullptr;
	locator = nullptr;

	vtkSmartPointer<vtkFloatArray> interpolatedElevation =
		  vtkFloatArray::SafeDownCast(interpolatedPoly->GetPointData()->GetArray("elevation"));

	for (int i = 0; i < width; i++){
		for (int j = 0; j < height; j++){
			float* pixel = static_cast<float*>(image->GetScalarPointer(i,j,0));
			vtkIdType cellId = interpolatedPoly->GetCell(j * width + i)->GetPointId(0);
			pixel[0] = interpolatedElevation->GetValue(cellId);
		}
	}

	log << "OK\nMedian filter...";

	vtkSmartPointer<vtkImageMedian3D> medianFilter =
			vtkSmartPointer<vtkImageMedian3D>::New();
	medianFilter->SetInputData(image);
	medianFilter->SetKernelSize(
			std::max(1.0, resolution),
			std::max(1.0, resolution),
			1);
	medianFilter->Update();

	log << "OK\n";

//	double diffuseIterations = std::max(1.0, resolution / 2.0);
//	vtkSmartPointer<vtkImageAnisotropicDiffusion2D> diffuse1 =
//			vtkSmartPointer<vtkImageAnisotropicDiffusion2D>::New();
//	diffuse1->SetInputConnection(medianFilter->GetOutputPort());
//	diffuse1->FacesOn();
//	diffuse1->EdgesOn();
//	diffuse1->CornersOn();
//	diffuse1->SetDiffusionFactor(1); // Full strength
//	diffuse1->GradientMagnitudeThresholdOn();
//	diffuse1->SetDiffusionThreshold(0.2); // Don't smooth jumps in elevation > than 0.20m
//	diffuse1->SetNumberOfIterations(diffuseIterations);
//	diffuse1->Update();

	if (outputDsmFile != ""){
		log << "Saving DSM to file... ";
		vtkSmartPointer<vtkTIFFWriter> tiffWriter =
				vtkSmartPointer<vtkTIFFWriter>::New();
		tiffWriter->SetFileName(outputDsmFile.c_str());
		tiffWriter->SetInputData(medianFilter->GetOutput());
		tiffWriter->Write();
		log << "OK\n";
	}

	log << "Triangulate... ";

	vtkSmartPointer<vtkGreedyTerrainDecimation> terrain =
			vtkSmartPointer<vtkGreedyTerrainDecimation>::New();
	terrain->SetErrorMeasureToNumberOfTriangles();
	terrain->SetNumberOfTriangles(maxVertexCount * 2); // Approximate
	terrain->SetInputData(medianFilter->GetOutput());
	terrain->BoundaryVertexDeletionOn();

	log << "OK\nTransform... ";
	vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
	transform->Translate(-extentX / 2.0 + centerX,
			-extentY / 2.0 + centerY, 0);
	transform->Scale(extentX / width, extentY / height, 1);

	vtkSmartPointer<vtkTransformFilter> transformFilter =
	    vtkSmartPointer<vtkTransformFilter>::New();
	transformFilter->SetInputConnection(terrain->GetOutputPort());
	transformFilter->SetTransform(transform);

	log << "OK\n";

	log << "Saving mesh to file... ";

	vtkSmartPointer<vtkPLYWriter> plyWriter =
			vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(outputFile.c_str());
	plyWriter->SetInputConnection(transformFilter->GetOutputPort());
	plyWriter->SetFileTypeToASCII();
	plyWriter->Write();

	log << "OK\n";

#ifdef SUPPORTDEBUGWINDOW
	if (showDebugWindow){
		vtkSmartPointer<vtkPolyDataMapper> mapper =
				vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(transformFilter->GetOutputPort());
		mapper->SetScalarRange(150, 170);

	//	  vtkSmartPointer<vtkDataSetMapper> mapper =
	//	    vtkSmartPointer<vtkDataSetMapper>::New();
	//	  mapper->SetInputData(image);
	//	  mapper->SetScalarRange(150, 170);

		  vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
		  actor->SetMapper(mapper);
		  actor->GetProperty()->SetPointSize(5);

		  vtkSmartPointer<vtkRenderer> renderer =
			vtkSmartPointer<vtkRenderer>::New();
		  vtkSmartPointer<vtkRenderWindow> renderWindow =
			vtkSmartPointer<vtkRenderWindow>::New();
		  renderWindow->AddRenderer(renderer);
		  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
			vtkSmartPointer<vtkRenderWindowInteractor>::New();
		  renderWindowInteractor->SetRenderWindow(renderWindow);

		  renderer->AddActor(actor);
		  renderer->SetBackground(0.1804,0.5451,0.3412); // Sea green

		  renderWindow->Render();
		  renderWindowInteractor->Start();
	}
#endif
}

void Odm25dMeshing::parseArguments(int argc, char **argv) {
	for (int argIndex = 1; argIndex < argc; ++argIndex) {
		// The argument to be parsed.
		std::string argument = std::string(argv[argIndex]);

		if (argument == "-help") {
			printHelp();
			exit(0);
		} else if (argument == "-verbose") {
			log.setIsPrintingInCout(true);
		} else if (argument == "-maxVertexCount" && argIndex < argc) {
            ++argIndex;
            if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            std::stringstream ss(argv[argIndex]);
            ss >> maxVertexCount;
            if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            maxVertexCount = std::max<unsigned int>(maxVertexCount, 0);
            log << "Vertex count was manually set to: " << maxVertexCount << "\n";
		} else if (argument == "-resolution" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> resolution;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");

			resolution = std::min<double>(100000, std::max<double>(resolution, 0));
			log << "Resolution was manually set to: " << resolution << "\n";
		} else if (argument == "-neighbors" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> neighbors;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
			neighbors = std::min<unsigned int>(1000, std::max<unsigned int>(neighbors, 1));
			log << "Neighbors was manually set to: " << neighbors << "\n";
		} else if (argument == "-inputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			inputFile = std::string(argv[argIndex]);
			std::ifstream testFile(inputFile.c_str(), std::ios::binary);
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument	+ "' has a bad value. (file not accessible)");
			}
			testFile.close();
			log << "Reading point cloud at: " << inputFile << "\n";
		} else if (argument == "-outputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			}
			outputFile = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing output to: " << outputFile << "\n";
		}else if (argument == "-outputDsmFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			outputDsmFile = std::string(argv[argIndex]);
			std::ofstream testFile(outputDsmFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument	+ "' has a bad value. (file not accessible)");
			}
			testFile.close();
			log << "Saving DSM output to: " << outputDsmFile << "\n";
		} else if (argument == "-showDebugWindow") {
			showDebugWindow = true;
		} else if (argument == "-logFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			logFilePath = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing log information to: " << logFilePath << "\n";
		} else {
			printHelp();
			throw Odm25dMeshingException("Unrecognised argument '" + argument + "'");
		}
	}
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	log << "Create a 2.5D mesh from a point cloud. "
		<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	log << "	-inputFile	<path>	to PLY point cloud\n"
		<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: " << outputFile << ")\n"
		<< "	-outputDsmFile	<path>	Optionally output the Digital Surface Model (DSM) computed for generating the mesh. (default: " << outputDsmFile << ")\n"
		<< "	-logFile	<path>	log file path (default: " << logFilePath << ")\n"
		<< "	-verbose	whether to print verbose output (default: " << (printInCoutPop ? "true" : "false") << ")\n"
		<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
		<< "	-neighbors	<1 - 1000>	Number of nearest neighbors to consider when doing shepard's interpolation and outlier removal. Higher values lead to smoother meshes but take longer to process. (default: " << neighbors << ")\n"
		<< "	-resolution	<0 - N>	Size of the interpolated digital surface model (DSM) used for deriving the 2.5D mesh, expressed in pixels per meter unit. When set to zero, the program automatically attempts to find a good value based on the point cloud extent and target vertex count. (default: " << resolution << ")\n"

		<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



