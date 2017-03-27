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

		log << "Done!" << "\n";

		// TODO

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

void Odm25dMeshing::parseArguments(int argc, char **argv) {
	for (int argIndex = 1; argIndex < argc; ++argIndex) {
		// The argument to be parsed.
		std::string argument = std::string(argv[argIndex]);

		if (argument == "-help") {
			printHelp();
		} else if (argument == "-verbose") {
			log.setIsPrintingInCout(true);
		} else if (argument == "-maxVertexCount" && argIndex < argc) {
//            ++argIndex;
//            if (argIndex >= argc)
//            {
//                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
//            }
//            std::stringstream ss(argv[argIndex]);
//            ss >> maxVertexCount_;
//            if (ss.bad())
//            {
//                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
//            }
//            log << "Vertex count was manually set to: " << maxVertexCount_ << "\n";
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

void Odm25dMeshing::loadPointCloud(){
	  std::vector<Pwn> points; // store points with normals
	  std::vector<Color> colors; // store colors in separate container

	  PlyInterpreter interpreter(points, colors);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points and normals from:\n" + inputFile + "\n");
	  }

	  log << "Found " << points.size() << " points \n";

	  for (std::size_t i = 0; i < points.size (); ++ i){
		  std::cout << points[i].first << std::endl;
	  }

//	if (pcl::io::loadPLYFile < pcl::PointNormal
//			> (inputFile_.c_str(), *points_.get()) == -1) {
//		throw OdmMeshingException(
//				"Error when reading points and normals from:\n" + inputFile_
//						+ "\n");
//	} else {
//		log << "Successfully loaded " << points_->size()
//				<< " points with corresponding normals from file.\n";
//	}
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "odm_25dmeshing\n\n";

	log << "Purpose:" << "\n";
	log
			<< "Create a 2.5D mesh from an oriented point cloud (points with normals) using a constrained delaunay triangulation"
			<< "\n";

	log << "Usage:" << "\n";
	log	<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional."
			<< "\n\n";

	log << "The following flags are available\n";
	log	<< "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
	log
			<< "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";

	log	<< "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: "
			<< "\n";
	log << "\"-inputFile <path>\" (mandatory)" << "\n";
	log	<< "\"Input ply file that must contain a point cloud with normals.\n\n";

	log << "\"-outputFile <path>\" (optional, default: odm_mesh.ply)" << "\n";
	log << "\"Target file in which the mesh is saved.\n\n";

	log << "\"-logFile <path>\" (optional, default: odm_25dmeshing_log.txt)"
			<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}


void Odm25dMeshing::writeMeshToPly() {
	log << "Saving mesh to file.\n";
//	if (pcl::io::savePLYFile(outputFile_.c_str(), *mesh_.get()) == -1) {
//		throw Odm25dMeshingException(
//				"Error when saving mesh to file:\n" + outputFile + "\n");
//	} else {
//		log << "Successfully wrote mesh to:\n" << outputFile << "\n";
//	}
	// TODO
}
