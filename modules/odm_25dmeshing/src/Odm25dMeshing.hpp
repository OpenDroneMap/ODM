#pragma once

//#define SUPPORTDEBUGWINDOW 1

#ifdef SUPPORTDEBUGWINDOW

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDataSetMapper.h>

#endif

#include <vtkShepardKernel.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkGreedyTerrainDecimation.h>
#include <vtkPLYWriter.h>
#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkOctreePointLocator.h>
#include <vtkPointInterpolator.h>
#include <vtkPlaneSource.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkImageAnisotropicDiffusion2D.h>
#include <vtkTIFFWriter.h>
#include <vtkStatisticalOutlierRemoval.h>
#include <vtkImageMedian3D.h>
#include <vtkRadiusOutlierRemoval.h>

// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

#include <cstring>
#include "tinyply.h"
using namespace tinyply;

#include "Logger.hpp"

class Odm25dMeshing {
public:
	Odm25dMeshing() :
			log(false) {};
	~Odm25dMeshing() {};

	/*!
	 * \brief   run     Runs the meshing functionality using the provided input arguments.
	 *                  For a list of accepted arguments, please see the main page documentation or
	 *                  call the program with parameter "-help".
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 * \return  0       If successful.
	 */
	int run(int argc, char **argv);

private:

	/*!
	 * \brief parseArguments    Parses command line arguments.
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 */
	void parseArguments(int argc, char** argv);

	/*!
	 * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
	 */
	void printHelp();

	void loadPointCloud();
	void buildMesh();

	Logger log;

	std::string inputFile = "";
	std::string outputFile = "odm_25dmesh.ply";
	std::string logFilePath = "odm_25dmeshing_log.txt";
	int maxVertexCount = 100000;
	double resolution = 0;
	unsigned int neighbors = 24;
	std::string outputDsmFile = "";
	bool showDebugWindow = false;

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
};

class Odm25dMeshingException: public std::exception {

public:
	Odm25dMeshingException() :
			message("Error in Odm25dMeshing") {
	}
	Odm25dMeshingException(std::string msgInit) :
			message("Error in Odm25dMeshing:\n" + msgInit) {
	}
	~Odm25dMeshingException() throw () {
	}
	virtual const char* what() const throw () {
		return message.c_str();
	}

private:
	std::string message; /**< The error message **/
};
