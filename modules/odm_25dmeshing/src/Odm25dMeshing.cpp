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
					0.0f);
			elevation->InsertNextValue(*(reinterpret_cast<double *>(point + posZ->offset)));
		} else if (posX->datatype == pcl::PCLPointField::FLOAT32) {
			points->InsertNextPoint(*(reinterpret_cast<float *>(point + posX->offset)),
								*(reinterpret_cast<float *>(point + posY->offset)),
								0.0f);
			elevation->InsertNextValue(*(reinterpret_cast<float *>(point + posZ->offset)));
		}
	}

	log << "Loaded " << points->GetNumberOfPoints() << " points\n";
}

void Odm25dMeshing::buildMesh(){
//	double bounds[6];
//	reader->GetOutput()->GetBounds(bounds);

//	vtkSmartPointer<vtkElevationFilter> elevationFilter =
//			vtkSmartPointer<vtkElevationFilter>::New();
//	elevationFilter->SetInputConnection(reader->GetOutputPort());
//	elevationFilter->SetLowPoint(0.0, 0.0, bounds[4]);
//	elevationFilter->SetHighPoint(0.0, 0.0, bounds[5]);
//	elevationFilter->SetScalarRange(0.0f, 1.0f);
//	elevationFilter->Update();

	vtkSmartPointer<vtkPolyData> polydataToProcess =
	  vtkSmartPointer<vtkPolyData>::New();
	polydataToProcess->SetPoints(points);
	polydataToProcess->GetPointData()->SetScalars(elevation);

	const float RESOLUTION = 10.0f; // pixels per meter
	const float RADIUS = 1.0f;

	double *bounds = polydataToProcess->GetBounds();

	float extentX = bounds[1] - bounds[0];
	float extentY = bounds[3] - bounds[2];

	int width = ceil(extentX * RESOLUTION);
	int height = ceil(extentY * RESOLUTION);

	vtkSmartPointer<vtkPlaneSource> plane =
			vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetResolution(width, height);
	plane->SetOrigin(0.0f, 0.0f, 0.0f);
	plane->SetPoint1(extentX, 0.0f, 0.0f);
	plane->SetPoint2(0.0f, extentY, 0);
	plane->SetCenter(polydataToProcess->GetCenter());
	plane->SetNormal(0.0f, 0.0f, 1.0f);

	vtkSmartPointer<vtkStaticPointLocator> locator =
			vtkSmartPointer<vtkStaticPointLocator>::New();
	locator->SetDataSet(polydataToProcess);
	locator->BuildLocator();

	vtkSmartPointer<vtkShepardKernel> shepardKernel =
				vtkSmartPointer<vtkShepardKernel>::New();
	shepardKernel->SetRadius(RADIUS);
	shepardKernel->SetPowerParameter(2.0);

	vtkSmartPointer<vtkPointInterpolator> interpolator =
				vtkSmartPointer<vtkPointInterpolator>::New();
	interpolator->SetInputConnection(plane->GetOutputPort());
	interpolator->SetSourceData(polydataToProcess);
	interpolator->SetKernel(shepardKernel);
	interpolator->SetLocator(locator);
	interpolator->SetNullPointsStrategyToClosestPoint();
	interpolator->Update();


	vtkSmartPointer<vtkPolyData> interpolatedPoly =
			interpolator->GetPolyDataOutput();
	  vtkSmartPointer<vtkFloatArray> interpolatedElevation =
			  vtkFloatArray::SafeDownCast(interpolatedPoly->GetPointData()->GetArray("elevation"));


	vtkSmartPointer<vtkImageData> image =
	    vtkSmartPointer<vtkImageData>::New();
	image->SetDimensions(width, height, 1);
	image->AllocateScalars(VTK_FLOAT, 1);
	for (int i = 0; i < width; i++){
		for (int j = 0; j < height; j++){
			float* pixel = static_cast<float*>(image->GetScalarPointer(i,j,0));
			vtkIdType cellId = interpolatedPoly->GetCell(j * width + i)->GetPointId(0);
			pixel[0] = interpolatedElevation->GetValue(cellId);
		}
	}

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
	    vtkSmartPointer<vtkVertexGlyphFilter>::New();
	  vertexFilter->SetInputData(interpolator->GetOutput());
	  vertexFilter->Update();
	  vtkSmartPointer<vtkPolyData> polydataToShow =
	    vtkSmartPointer<vtkPolyData>::New();
	  polydataToShow->ShallowCopy(vertexFilter->GetOutput());

	vtkSmartPointer<vtkGreedyTerrainDecimation> decimation =
			vtkSmartPointer<vtkGreedyTerrainDecimation>::New();
//	decimation->SetErrorMeasureToNumberOfTriangles();
//	decimation->SetNumberOfTriangles(100000);
	decimation->SetInputData(image);
	decimation->Update();

	vtkSmartPointer<vtkPLYWriter> plyWriter =
			vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(outputFile.c_str());
	plyWriter->SetInputConnection(decimation->GetOutputPort());
	plyWriter->Write();

	vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(decimation->GetOutputPort());
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
	  actor->GetProperty()->SetInterpolationToFlat();
//	  actor->GetProperty()->EdgeVisibilityOn();
//	  actor->GetProperty()->SetEdgeColor(1,0,0);

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

//	// Create a set of vertices (polydata)
//	vtkSmartPointer<vtkPoints> points =
//	  vtkSmartPointer<vtkPoints>::New();
//	points->InsertNextPoint(100.0, 0.0, 0.0);
//	points->InsertNextPoint(300.0, 0.0, 0.0);
//
//	// Setup colors
//	unsigned char white[3] = {255, 255, 255};
//	unsigned char black[3] = {0, 0, 0};
//
//	vtkSmartPointer<vtkUnsignedCharArray> vertexColors =
//	vtkSmartPointer<vtkUnsignedCharArray>::New();
//	vertexColors->SetNumberOfComponents(3);
//	vertexColors->SetName("Colors");
//	vertexColors->InsertNextTupleValue(black);
//	vertexColors->InsertNextTupleValue(white);
//
//	// Create a scalar array for the pointdata, each value represents the distance
//	// of the vertices from the first vertex
//	vtkSmartPointer<vtkFloatArray> values =
//	  vtkSmartPointer<vtkFloatArray>::New();
//	values->SetNumberOfComponents(1);
//	values->SetName("Values");
//	values->InsertNextValue(0.0);
//	values->InsertNextValue(1.0);
//
//	// We must make two objects, because the ShepardMethod uses the ActiveScalars, as does the renderer!
//	vtkSmartPointer<vtkPolyData> polydataToProcess =
//	  vtkSmartPointer<vtkPolyData>::New();
//	polydataToProcess->SetPoints(points);
//	polydataToProcess->GetPointData()->SetScalars(values);
//
//	vtkSmartPointer<vtkPolyData> polydataToVisualize =
//	vtkSmartPointer<vtkPolyData>::New();
//	polydataToVisualize->SetPoints(points);
//	polydataToVisualize->GetPointData()->SetScalars(vertexColors);
//
//	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter =
//	vtkSmartPointer<vtkVertexGlyphFilter>::New();
//	#if VTK_MAJOR_VERSION <= 5
//	vertexGlyphFilter->AddInputConnection(polydataToVisualize->GetProducerPort());
//	#else
//	vertexGlyphFilter->AddInputData(polydataToVisualize);
//	#endif
//	vertexGlyphFilter->Update();
//
//	//Create a mapper and actor
//	vtkSmartPointer<vtkPolyDataMapper> vertsMapper =
//	  vtkSmartPointer<vtkPolyDataMapper>::New();
//	//vertsMapper->ScalarVisibilityOff();
//	vertsMapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());
//
//	vtkSmartPointer<vtkActor> vertsActor =
//	vtkSmartPointer<vtkActor>::New();
//	vertsActor->SetMapper(vertsMapper);
//	vertsActor->GetProperty()->SetColor(1,0,0);
//	vertsActor->GetProperty()->SetPointSize(3);
//
//	// Create a shepard filter to interpolate the vertices over a regularized image grid
//	vtkSmartPointer<vtkShepardMethod> shepard = vtkSmartPointer<vtkShepardMethod>::New();
//	#if VTK_MAJOR_VERSION <= 5
//	shepard->SetInputConnection(polydataToProcess->GetProducerPort());
//	#else
//	shepard->SetInputData(polydataToProcess);
//	#endif
//	shepard->SetSampleDimensions(2,2,2);
//	shepard->SetModelBounds(100,300,-10,10,-10,10);
//	shepard->SetMaximumDistance(1);
//
//	// Contour the shepard generated image at 3 isovalues
//	// The accuracy of the results are highly dependent on how the shepard filter is set up
//	vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
//	contourFilter->SetNumberOfContours(3);
//	contourFilter->SetValue(0, 0.25);
//	contourFilter->SetValue(1, 0.50);
//	contourFilter->SetValue(2, 0.75);
//	contourFilter->SetInputConnection(shepard->GetOutputPort());
//	contourFilter->Update();
//
//	//Create a mapper and actor for the resulting isosurfaces
//	vtkSmartPointer<vtkPolyDataMapper> contourMapper =
//	vtkSmartPointer<vtkPolyDataMapper>::New();
//	contourMapper->SetInputConnection(contourFilter->GetOutputPort());
//	contourMapper->ScalarVisibilityOn();
//	contourMapper->SetColorModeToMapScalars();
//
//	vtkSmartPointer<vtkActor> contourActor =
//	vtkSmartPointer<vtkActor>::New();
//	contourActor->SetMapper(contourMapper);
//	contourActor->GetProperty()->SetAmbient(1);
//	contourActor->GetProperty()->SetSpecular(0);
//	contourActor->GetProperty()->SetDiffuse(0);
//
//	// Report the results of the interpolation
//	double *range = contourFilter->GetOutput()->GetScalarRange();
//
//	std::cout << "Shepard interpolation:" << std::endl;
//	std::cout << "contour output scalar range: " << range[0] << ", " << range[1] << std::endl;
//
//	vtkIdType nCells = contourFilter->GetOutput()->GetNumberOfCells();
//	double bounds[6];
//	for( vtkIdType i = 0; i < nCells; ++i )
//	{
//	if(i%2) // each isosurface value only has 2 cells to report on the odd ones
//	{
//	  contourFilter->GetOutput()->GetCellBounds(i,bounds);
//	  std::cout << "cell " << i << ", x position: " << bounds[0] << std::endl;
//	}
//	}
//
//	// Create a transfer function to color the isosurfaces
//	vtkSmartPointer<vtkColorTransferFunction> lut =
//	vtkSmartPointer<vtkColorTransferFunction>::New();
//	lut->SetColorSpaceToRGB();
//	lut->AddRGBPoint(range[0],0,0,0);//black
//	lut->AddRGBPoint(range[1],1,1,1);//white
//	lut->SetScaleToLinear();
//
//	contourMapper->SetLookupTable( lut );
//
//	// Create a renderer, render window and interactor
//	vtkSmartPointer<vtkRenderer> renderer =
//	vtkSmartPointer<vtkRenderer>::New();
//	renderer->GradientBackgroundOn();
//	renderer->SetBackground(0,0,1);
//	renderer->SetBackground2(1,0,1);
//
//	vtkSmartPointer<vtkRenderWindow> renderWindow =
//	vtkSmartPointer<vtkRenderWindow>::New();
//	renderWindow->AddRenderer(renderer);
//	renderer->AddActor(contourActor);
//	renderer->AddActor(vertsActor);
//
//	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//	vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	renderWindowInteractor->SetRenderWindow(renderWindow);
//
//	// Position the camera so that the image produced is viewable
//	vtkCamera* camera = renderer->GetActiveCamera();
//	camera->SetPosition(450, 100, 100);
//	camera->SetFocalPoint(200, 0, 0);
//	camera->SetViewUp(0, 0, 1);
//
//	renderWindowInteractor->Start();
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
//		} else if (argument == "-maxVertexCount" && argIndex < argc) {
//            ++argIndex;
//            if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
//            std::stringstream ss(argv[argIndex]);
//            ss >> maxVertexCount;
//            if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
//            maxVertexCount = std::max<unsigned int>(maxVertexCount, 0);
//            log << "Vertex count was manually set to: " << maxVertexCount << "\n";
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
//		<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
//		<< "	-wlopIterations	<1 - 1000>	Iterations of the Weighted Locally Optimal Projection (WLOP) simplification algorithm. Higher values take longer but produce a smoother mesh. (default: " << wlopIterations << ")\n"

		<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



