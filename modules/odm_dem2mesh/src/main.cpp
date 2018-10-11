#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include "CmdLineParser.h"
#include "Logger.h"
#include <vtkSmartPointer.h>
#include <vtkGreedyTerrainDecimation.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkPLYWriter.h>
#include <vtkAdaptiveSubdivisionFilter.h>
#include <vtkImageData.h>

#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()

Logger logWriter;

typedef struct Point2D{
   double x;
   double y;

   Point2D(): x(0.0), y(0.0){}
   Point2D(double x, double y): x(x), y(y){}
} Point2D;

typedef struct BoundingBox{
    Point2D max;
    Point2D min;

    BoundingBox(): max(Point2D()), min(Point2D()){}
    BoundingBox(Point2D min, Point2D max): max(max), min(min){}
} BoundingBox;

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
    OutputFile( "outputFile" );
cmdLineParameter< int >
    MaxVertexCount( "maxVertexCount" );
cmdLineReadable
	Verbose( "verbose" );

cmdLineReadable* params[] = {
    &InputFile , &OutputFile , &MaxVertexCount , &Verbose ,
    NULL
};

void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input DSM raster>" << std::endl
              << "\t -" << OutputFile.name << " <output PLY mesh>" << std::endl
              << "\t [-" << MaxVertexCount.name << " <target number vertices> (Default: 100000)]" << std::endl
              << "\t [-" << Verbose.name << "]" << std::endl;
    exit(EXIT_FAILURE);
}


void logArgs(cmdLineReadable* params[], Logger& logWriter){
    logWriter("Running with parameters:\n");
    char str[1024];
    for( int i=0 ; params[i] ; i++ ){
        if( params[i]->set ){
            params[i]->writeValue( str );
            if( strlen( str ) ) logWriter( "\t-%s %s\n" , params[i]->name , str );
            else                logWriter( "\t-%s\n" , params[i]->name );
        }
    }
}

Point2D geoLoc(float xloc, float yloc, double *affine){
    return Point2D(affine[0] + xloc*affine[1] + yloc*affine[2], affine[3] + xloc*affine[4] + yloc*affine[5]);
}

BoundingBox getExtent(GDALDataset *dataset){
    double affine[6];
    dataset->GetGeoTransform(affine);
    return BoundingBox(geoLoc(0, dataset->GetRasterYSize(), affine), geoLoc(dataset->GetRasterXSize(), 0, affine));
}

int main(int argc, char **argv) {
    cmdLineParse( argc-1 , &argv[1] , params );
    if( !InputFile.set || !OutputFile.set ) help(argv[0]);
    if ( !MaxVertexCount.set ) MaxVertexCount.value = 100000;

    logWriter.verbose = Verbose.set;
    logArgs(params, logWriter);


    GDALDataset  *dataset;
    GDALAllRegister();
    dataset = (GDALDataset *) GDALOpen( InputFile.value, GA_ReadOnly );
    if( dataset != NULL )
    {
        int width = dataset->GetRasterXSize();
        int height = dataset->GetRasterYSize();

        logWriter("Raster Size is %dx%d\n", width, height);
        BoundingBox extent = getExtent(dataset);

        logWriter("Extent is (%f, %f), (%f, %f)\n", extent.min.x, extent.max.x, extent.min.y, extent.max.y);

        double extentX = extent.max.x - extent.min.x;
        double extentY = extent.max.y - extent.min.y;

        vtkSmartPointer<vtkImageData> image =
          vtkSmartPointer<vtkImageData>::New();
        image->SetDimensions(width, height, 1);
        image->AllocateScalars(VTK_FLOAT, 1);

        GDALRasterBand *band = dataset->GetRasterBand(1);
        float *rasterData = new float[width];

        for (int y = 0; y < height; y++){
            if (band->RasterIO( GF_Read, 0, height - y - 1, width, 1,
                                rasterData, width, 1, GDT_Float32, 0, 0 ) == CE_Failure){
                std::cerr << "Cannot access raster data" << std::endl;
                exit(1);
            }

            for (int x = 0; x < width; x++){
                (static_cast<float*>(image->GetScalarPointer(x,y,0)))[0] = rasterData[x];
            }
        }

        GDALClose(dataset);
        delete[] rasterData;

        vtkSmartPointer<vtkGreedyTerrainDecimation> terrain =
              vtkSmartPointer<vtkGreedyTerrainDecimation>::New();
        terrain->SetErrorMeasureToNumberOfTriangles();
        terrain->SetNumberOfTriangles(MaxVertexCount.value * 2); // Approximate
        terrain->SetInputData(image);
        terrain->BoundaryVertexDeletionOn();

        vtkSmartPointer<vtkAdaptiveSubdivisionFilter> subdivision =
                vtkSmartPointer<vtkAdaptiveSubdivisionFilter>::New();
        subdivision->SetMaximumTriangleArea(50);
        subdivision->SetInputConnection(terrain->GetOutputPort());

        logWriter("OK\nTransform... ");
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        transform->Scale(extentX / width, extentY / height, 1);
        transform->Translate(extent.min.x * (width / extentX), extent.min.y * (height / extentY), 0);

        vtkSmartPointer<vtkTransformFilter> transformFilter =
          vtkSmartPointer<vtkTransformFilter>::New();
        transformFilter->SetInputConnection(subdivision->GetOutputPort());
        transformFilter->SetTransform(transform);

        logWriter("OK\nSaving mesh to file... ");

        vtkSmartPointer<vtkPLYWriter> plyWriter =
              vtkSmartPointer<vtkPLYWriter>::New();
        plyWriter->SetFileName(OutputFile.value);
        plyWriter->SetInputConnection(transformFilter->GetOutputPort());
        plyWriter->SetFileTypeToBinary();
        plyWriter->Write();

        logWriter("OK\n");
    }else{
        std::cerr << "Cannot open " << InputFile.value << std::endl;
    }
}
