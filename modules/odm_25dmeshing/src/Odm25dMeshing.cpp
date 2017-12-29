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
	elevation->SetName("elevation");
	elevation->SetNumberOfComponents(1);

	pcl::PCLPointCloud2 blob;

	log << "Loading point cloud... ";

	if (pcl::io::loadPLYFile(inputFile.c_str(), blob) == -1) {
		throw Odm25dMeshingException("Error when reading from: " + inputFile);
	}

	log << "OK\n";

	log << "Scanning fields... ";

	pcl::PCLPointField *posX = NULL, *posY = NULL, *posZ = NULL;

#define ASSIGN(_name, _field) if (blob.fields[i].name == _name){ _field = &blob.fields[i]; log << _name << " "; continue; }

	for (size_t i = 0; i < blob.fields.size(); ++i) {
		ASSIGN("x", posX);
		ASSIGN("y", posY);
		ASSIGN("z", posZ);
	}

	log << "OK\n";

	if (posX == NULL || posY == NULL || posZ == NULL)
		throw Odm25dMeshingException(
				"Position attributes (x,y,z) missing from input");
	if (posX->datatype != pcl::PCLPointField::FLOAT32
			&& posX->datatype != pcl::PCLPointField::FLOAT64)
		throw Odm25dMeshingException(
				"Only float and float64 types are supported for position information");


	for (size_t point_step = 0, i = 0; point_step < blob.data.size();
			point_step += blob.point_step, i++) {
		uint8_t *point = blob.data.data() + point_step;

		if (posX->datatype == pcl::PCLPointField::FLOAT64) {
			points->InsertNextPoint(*(reinterpret_cast<double *>(point + posX->offset)),
					*(reinterpret_cast<double *>(point + posY->offset)),
					0.0);
			elevation->InsertNextValue(*(reinterpret_cast<double *>(point + posZ->offset)));
		} else if (posX->datatype == pcl::PCLPointField::FLOAT32) {
			points->InsertNextPoint(*(reinterpret_cast<float *>(point + posX->offset)),
								*(reinterpret_cast<float *>(point + posY->offset)),
								0.0);
			elevation->InsertNextValue(*(reinterpret_cast<float *>(point + posZ->offset)));
		}
	}

	log << "Loaded " << points->GetNumberOfPoints() << " points\n";
}

void Odm25dMeshing::buildMesh(){
	vtkSmartPointer<vtkPolyData> polydataToProcess =
	  vtkSmartPointer<vtkPolyData>::New();
	polydataToProcess->SetPoints(points);
	polydataToProcess->GetPointData()->SetScalars(elevation);

	const double RADIUS = 0.1;
	const int RADIUS_STEPS = 1;
	const float NODATA = -9999;

	double *bounds = polydataToProcess->GetBounds();
	double *center = polydataToProcess->GetCenter();

	double extentX = bounds[1] - bounds[0];
	double extentY = bounds[3] - bounds[2];

	int width = ceil(extentX * resolution);
	int height = ceil(extentY * resolution);

	log << "Plane extentX: " << extentX <<
				", extentY: " << extentY << "\n";

	vtkSmartPointer<vtkPlaneSource> plane =
			vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetResolution(width, height);
	plane->SetOrigin(0.0, 0.0, 0.0);
	plane->SetPoint1(extentX, 0.0, 0.0);
	plane->SetPoint2(0.0, extentY, 0);
	plane->SetCenter(center);
	plane->SetNormal(0.0, 0.0, 1.0);

	vtkSmartPointer<vtkStaticPointLocator> locator =
			vtkSmartPointer<vtkStaticPointLocator>::New();
	locator->SetDataSet(polydataToProcess);
	locator->BuildLocator();

	vtkSmartPointer<vtkShepardKernel> shepardKernel =
				vtkSmartPointer<vtkShepardKernel>::New();
	shepardKernel->SetPowerParameter(2.0);

	vtkSmartPointer<vtkImageData> image =
	    vtkSmartPointer<vtkImageData>::New();
	image->SetDimensions(width, height, 1);

	log << "DSM size is " << width << "x" << height << " (" << ceil(width * height * sizeof(float) * 1e-6) << " MB) \n";

	image->AllocateScalars(VTK_FLOAT, 1);
	for (int i = 0; i < width; i++){
		for (int j = 0; j < height; j++){
			float* pixel = static_cast<float*>(image->GetScalarPointer(i,j,0));
			pixel[0] = NODATA;
		}
	}

	log << "Point interpolation using shepard's kernel...";

	vtkSmartPointer<vtkPointInterpolator> interpolator =
				vtkSmartPointer<vtkPointInterpolator>::New();
	interpolator->SetInputConnection(plane->GetOutputPort());
	interpolator->SetSourceData(polydataToProcess);
	interpolator->SetKernel(shepardKernel);
	interpolator->SetLocator(locator);
	interpolator->SetNullValue(NODATA);

	double currentRadius = RADIUS;
	const double sqrt_2 = sqrt(2.0);

	for (int r = 0; r < RADIUS_STEPS; r++, currentRadius *= sqrt_2){
		log << " " << (r + 1) << " (" << currentRadius << ")...";

		shepardKernel->SetRadius(currentRadius);

		// At the last step we linearly interpolate remaining values
		if (r == RADIUS_STEPS - 1) {
			interpolator->SetNullPointsStrategyToClosestPoint();
		}

		interpolator->Update();

		vtkSmartPointer<vtkPolyData> interpolatedPoly =
				interpolator->GetPolyDataOutput();

		  vtkSmartPointer<vtkFloatArray> interpolatedElevation =
				  vtkFloatArray::SafeDownCast(interpolatedPoly->GetPointData()->GetArray("elevation"));

		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				float* pixel = static_cast<float*>(image->GetScalarPointer(i,j,0));
				if (pixel[0] == NODATA){
					vtkIdType cellId = interpolatedPoly->GetCell(j * width + i)->GetPointId(0);
					pixel[0] = interpolatedElevation->GetValue(cellId);
				}
			}
		}
	}

	vtkSmartPointer<vtkImageAnisotropicDiffusion2D> diffusion =
			vtkSmartPointer<vtkImageAnisotropicDiffusion2D>::New();
	diffusion->SetInputData(image);
	diffusion->FacesOn();
	diffusion->EdgesOn();
	diffusion->CornersOn();
	diffusion->SetDiffusionFactor(0.5); // Half strength
	diffusion->GradientMagnitudeThresholdOn();
	diffusion->SetDiffusionThreshold(0.5); // Don't smooth jumps in elevation > than 0.3m
	diffusion->SetNumberOfIterations(5);
	diffusion->Update();

	log << " OK\nTriangulate... ";

	vtkSmartPointer<vtkGreedyTerrainDecimation> decimation =
			vtkSmartPointer<vtkGreedyTerrainDecimation>::New();
	decimation->SetErrorMeasureToNumberOfTriangles();
	decimation->SetNumberOfTriangles(maxVertexCount * 2); // Approximate
	decimation->SetInputData(diffusion->GetOutput());
	decimation->Update();

	log << "OK\nTransform... ";
	vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
	transform->Translate(-extentX / 2.0 + center[0],
			-extentY / 2.0 + center[1], 0);
	transform->Scale(extentX / width, extentY / height, 1);

	vtkSmartPointer<vtkTransformFilter> transformFilter =
	    vtkSmartPointer<vtkTransformFilter>::New();
	transformFilter->SetInputConnection(decimation->GetOutputPort());
	transformFilter->SetTransform(transform);

	log << "OK\n";

	vtkSmartPointer<vtkWindowedSincPolyDataFilter> smoother =
			vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
	smoother->SetInputConnection(transformFilter->GetOutputPort());
	smoother->SetNumberOfIterations(10);
	smoother->BoundarySmoothingOff();
	smoother->SetPassBand(0.5);
	smoother->Update();

	log << "Saving mesh to file...";

	vtkSmartPointer<vtkPLYWriter> plyWriter =
			vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(outputFile.c_str());
	plyWriter->SetInputConnection(smoother->GetOutputPort());
	plyWriter->SetFileTypeToASCII();
	plyWriter->Write();

	log << "OK\n";

#ifdef SHOWDEBUGWINDOW
	vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(smoother->GetOutputPort());
//	mapper->SetInputConnection(interpolator->GetOutputPort());
//	mapper->SetInputData(polydataToShow);
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

			resolution = std::min<double>(100000, std::max<double>(resolution, 0.00001));
			log << "Resolution was manually set to: " << resolution << "\n";
//		} else if (argument == "-outliersRemovalPercentage" && argIndex < argc) {
//			++argIndex;
//			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
//			std::stringstream ss(argv[argIndex]);
//			ss >> outliersRemovalPercentage;
//			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
//
//			outliersRemovalPercentage = std::min<double>(99.99, std::max<double>(outliersRemovalPercentage, 0));
//			log << "Outliers removal was manually set to: " << outliersRemovalPercentage << "\n";
//		} else if (argument == "-wlopIterations" && argIndex < argc) {
//			++argIndex;
//			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
//			std::stringstream ss(argv[argIndex]);
//			ss >> wlopIterations;
//			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
//
//			wlopIterations = std::min<unsigned int>(1000, std::max<unsigned int>(wlopIterations, 1));
//			log << "WLOP iterations was manually set to: " << wlopIterations << "\n";
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
	log << "Create a 2.5D mesh from an oriented point cloud (points with normals) using a constrained delaunay triangulation. "
		<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	log << "	-inputFile	<path>	to PLY point cloud\n"
		<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: " << outputFile << ")\n"
		<< "	-logFile	<path>	log file path (default: " << logFilePath << ")\n"
		<< "	-verbose	whether to print verbose output (default: " << (printInCoutPop ? "true" : "false") << ")\n"
		<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
		//		<< "	-wlopIterations	<1 - 1000>	Iterations of the Weighted Locally Optimal Projection (WLOP) simplification algorithm. Higher values take longer but produce a smoother mesh. (default: " << wlopIterations << ")\n"
		<< "	-resolution	<1 - N>	Size of the interpolated digital surface model (DSM) used for deriving the 2.5D mesh, expressed in pixels per meter unit. (default: " << resolution << ")\n"

		<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



