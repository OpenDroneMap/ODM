#include <iostream>
#include <string>
#include <fstream>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include "CmdLineParser.h"
#include "Logger.h"

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

Logger logWriter;

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
	OutputFile( "outputFile" );
cmdLineReadable
	Verbose( "verbose" );

cmdLineReadable* params[] =
{
    &InputFile , &OutputFile , &Verbose ,
    NULL
};

// TODO:
// 2. optional decimation filter
// 3. optional island removal


void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input polygon mesh>" << std::endl
              << "\t -" << OutputFile.name << " <output polygon mesh>" << std::endl
              << "\t [-" << Verbose.name << "]" << std::endl;
    exit(EXIT_FAILURE);
}


void logParams(){
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
    if( !InputFile.set || !OutputFile.set ) help( argv[0] );

    logWriter.verbose = Verbose.set;
    logWriter.outputFile = "odm_cleanmesh_log.txt";
    logParams();

	vtkSmartPointer<vtkPLYReader> reader =
    vtkSmartPointer<vtkPLYReader>::New();
        reader->SetFileName ( InputFile.value );

    vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter = 
    vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
	connectivityFilter->SetInputConnection(reader->GetOutputPort());
	connectivityFilter->SetExtractionModeToLargestRegion(); 

    logWriter("Saving cleaned mesh to file... \n");

    vtkSmartPointer<vtkPLYWriter> plyWriter =
        vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName(OutputFile.value);
    plyWriter->SetInputConnection(connectivityFilter->GetOutputPort());
    plyWriter->SetFileTypeToBinary();
    plyWriter->Write();

    logWriter("OK\n");
}
