#include <iostream>
#include <string>
#include <fstream>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkAlgorithmOutput.h>
#include <vtkQuadricDecimation.h>
#include "CmdLineParser.h"
#include "Logger.h"

Logger logWriter;

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
    OutputFile( "outputFile" );
cmdLineParameter< int >
    DecimateMesh( "decimateMesh" );
cmdLineReadable
    RemoveIslands( "removeIslands" ) ,
	Verbose( "verbose" );

cmdLineReadable* params[] = {
    &InputFile , &OutputFile , &DecimateMesh, &RemoveIslands, &Verbose ,
    NULL
};

void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input polygon mesh>" << std::endl
              << "\t -" << OutputFile.name << " <output polygon mesh>" << std::endl
              << "\t [-" << DecimateMesh.name << " <target number of vertices>]" << std::endl
              << "\t [-" << RemoveIslands.name << "]" << std::endl

              << "\t [-" << Verbose.name << "]" << std::endl;
    exit(EXIT_FAILURE);
}


void logArgs(cmdLineReadable* params[], Logger& logWriter){
    logWriter("Running with parameters:\n");
    char str[1024];
    for( int i=0 ; params[i] ; i++ ){
        if( params[i]->set ){
            params[i]->writeValue( str );
            if( strlen( str ) ) logWriter( "\t--%s %s\n" , params[i]->name , str );
            else                logWriter( "\t--%s\n" , params[i]->name );
        }
    }
}


int main(int argc, char **argv) {
    cmdLineParse( argc-1 , &argv[1] , params );
    if( !InputFile.set || !OutputFile.set ) help(argv[0]);
    if( !RemoveIslands.set && !DecimateMesh.set ) help (argv[0]);


    logWriter.verbose = Verbose.set;
    // logWriter.outputFile = "odm_cleanmesh_log.txt";
    logArgs(params, logWriter);

	vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
        reader->SetFileName ( InputFile.value );
    reader->Update();

    vtkPolyData *nextOutput = reader->GetOutput();

    vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter =
    vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
    connectivityFilter->SetExtractionModeToLargestRegion();

    vtkSmartPointer<vtkQuadricDecimation> decimationFilter =
    vtkSmartPointer<vtkQuadricDecimation>::New();

    if (RemoveIslands.set){
        logWriter("Removing islands\n");
        connectivityFilter->SetInputData(nextOutput);
        connectivityFilter->Update();
        nextOutput = connectivityFilter->GetOutput();
    }

    if (DecimateMesh.set){
        logWriter("Decimating mesh\n");

        int vertexCount = nextOutput->GetNumberOfPoints();
        logWriter("Current vertex count: %d\n", vertexCount);
        logWriter("Wanted vertex count: %d\n", DecimateMesh.value);

        if (vertexCount > DecimateMesh.value){
            double targetReduction = 1.0 - static_cast<double>(DecimateMesh.value) / static_cast<double>(vertexCount);
            logWriter("Target reduction set to %f\n", targetReduction);
            decimationFilter->SetTargetReduction(targetReduction);
            decimationFilter->SetInputData(nextOutput);
            decimationFilter->Update();
            nextOutput = decimationFilter->GetOutput();
        }else{
            logWriter("Skipping decimation\n");
        }
    }

    logWriter("Saving cleaned mesh to file... \n");

    vtkSmartPointer<vtkPLYWriter> plyWriter =
        vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName(OutputFile.value);
    plyWriter->SetFileTypeToBinary();
    plyWriter->SetInputData(nextOutput);
    plyWriter->Write();

    logWriter("OK\n");
}
