#include <iostream>
#include <string>
#include <fstream>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

std::string inputFile = "", outputFile = "";
bool verbose = false;

// TODO:
// 1. better help
// 2. optional decimation filter
// 3. optional island removal
// 4. exit macros

void help(){
	std::cout << "HELP TODO" << std::endl;
	// log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	// log << "Create a 2.5D mesh from a point cloud. "
	// 	<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	// log << "	-inputFile	<path>	to PLY point cloud\n"
	// 	<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: " << outputFile << ")\n"
	// 	<< "	-outputDsmFile	<path>	Optionally output the Digital Surface Model (DSM) computed for generating the mesh. (default: " << outputDsmFile << ")\n"
	// 	<< "	-logFile	<path>	log file path (default: " << logFilePath << ")\n"
	// 	<< "	-verbose	whether to print verbose output (default: " << (printInCoutPop ? "true" : "false") << ")\n"
	// 	<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
	// 	<< "	-neighbors	<1 - 1000>	Number of nearest neighbors to consider when doing shepard's interpolation and outlier removal. Higher values lead to smoother meshes but take longer to process. (default: " << neighbors << ")\n"
	// 	<< "	-resolution	<0 - N>	Size of the interpolated digital surface model (DSM) used for deriving the 2.5D mesh, expressed in pixels per meter unit. When set to zero, the program automatically attempts to find a good value based on the point cloud extent and target vertex count. (default: " << resolution << ")\n"

	// 	<< "\n";
	exit(0);
}

void parseArguments(int argc, char **argv) {
	for (int argIndex = 1; argIndex < argc; ++argIndex) {
		// The argument to be parsed.
		std::string argument = std::string(argv[argIndex]);

		if (argument == "-help") {
			help();
			exit(0);
		} else if (argument == "-verbose") {
			verbose = true;
		} else if (argument == "-inputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw std::runtime_error(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			inputFile = std::string(argv[argIndex]);
		} else if (argument == "-outputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw std::runtime_error(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			outputFile = std::string(argv[argIndex]);
		} else {
			help();
		}
	}
}


int main(int argc, char **argv) {
	parseArguments(argc, argv);
	if (inputFile.empty() || outputFile.empty()) help();

	vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
  		reader->SetFileName ( inputFile.c_str() );

    vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = 
    vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
	connectivityFilter->SetInputConnection(reader->GetOutputPort());
	connectivityFilter->SetExtractionModeToLargestRegion(); 

    std::cout << "Saving cleaned mesh to file... " << std::endl;

    vtkSmartPointer<vtkPLYWriter> plyWriter =
        vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName(outputFile.c_str());
    plyWriter->SetInputConnection(connectivityFilter->GetOutputPort());
    plyWriter->SetFileTypeToBinary();
    plyWriter->Write();

	std::cout << "OK" << std::endl;	
}
