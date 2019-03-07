#include <iostream>
#include <pdal/filters/OutlierFilter.hpp>
#include <pdal/filters/RangeFilter.hpp>
#include "CmdLineParser.h"
#include "Logger.h"
#include "FloatPlyReader.hpp"
#include "ModifiedPlyWriter.hpp"

Logger logWriter;

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
    OutputFile( "outputFile" );
cmdLineParameter< float >
    StandardDeviation( "sd" ) ,
    MeanK ( "meank" );
cmdLineReadable
	Verbose( "verbose" );

cmdLineReadable* params[] = {
    &InputFile , &OutputFile , &StandardDeviation, &MeanK, &Verbose ,
    NULL
};

void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input PLY point cloud>" << std::endl
              << "\t -" << OutputFile.name << " <output PLY point cloud>" << std::endl
              << "\t [-" << StandardDeviation.name << " <standard deviation threshold>]" << std::endl
              << "\t [-" << MeanK.name << " <mean number of neighbors >]" << std::endl

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
    if( !StandardDeviation.set ) StandardDeviation.value = 2.0;
    if( !MeanK.set ) MeanK.value = 8;

    logWriter.verbose = Verbose.set;
    logArgs(params, logWriter);

    logWriter("Filtering point cloud...\n");

    pdal::Options inPlyOpts;
    inPlyOpts.add("filename", InputFile.value);

    pdal::PointTable table;
    pdal::FloatPlyReader plyReader;
    plyReader.setOptions(inPlyOpts);

    pdal::Options outlierOpts;
    outlierOpts.add("method", "statistical");
    outlierOpts.add("mean_k", MeanK.value);
    outlierOpts.add("multiplier", StandardDeviation.value);

    pdal::OutlierFilter outlierFilter;
    outlierFilter.setInput(plyReader);
    outlierFilter.setOptions(outlierOpts);

    pdal::Options rangeOpts;
    rangeOpts.add("limits", "Classification![7:7]"); // Remove outliers

    pdal::RangeFilter rangeFilter;
    rangeFilter.setInput(outlierFilter);
    rangeFilter.setOptions(rangeOpts);

    pdal::Options outPlyOpts;
    outPlyOpts.add("storage_mode", "little endian");
    outPlyOpts.add("filename", OutputFile.value);

    pdal::ModifiedPlyWriter plyWriter;
    plyWriter.setOptions(outPlyOpts);
    plyWriter.setInput(rangeFilter);
    plyWriter.prepare(table);
    plyWriter.execute(table);

    logWriter("Done!\n");
}
