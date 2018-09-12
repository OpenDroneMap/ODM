#include <iostream>
#include <string>
#include <fstream>
#include "CmdLineParser.h"
#include "Logger.h"

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

struct PlyPoint{
    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;
} p;
size_t psize = sizeof(float) * 6;

float *rasterData;
int arr_width, arr_height;

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
    OutputFile( "outputFile" );
cmdLineParameter< float >
    SkirtHeightThreshold( "skirtHeightThreshold" ),
    SkirtIncrements("skirtIncrements"),
    SkirtHeightCap( "skirtHeightCap");
cmdLineReadable
	Verbose( "verbose" );

cmdLineReadable* params[] = {
    &InputFile , &OutputFile , &SkirtHeightThreshold, &SkirtIncrements, &SkirtHeightCap, &Verbose ,
    NULL
};

void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input DSM raster>" << std::endl
              << "\t -" << OutputFile.name << " <output PLY points>" << std::endl
              << "\t [-" << SkirtHeightThreshold.name << " <Height threshold between cells that triggers the creation of a skirt>]" << std::endl
              << "\t [-" << SkirtIncrements.name << " <Skirt height increments when adding a new skirt>]" << std::endl
              << "\t [-" << SkirtHeightCap.name << " <Height cap that blocks the creation of a skirt>]" << std::endl
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

Point2D geoLoc(float xloc, float yloc, double *affine){
    return Point2D(affine[0] + xloc*affine[1] + yloc*affine[2], affine[3] + xloc*affine[4] + yloc*affine[5]);
}

BoundingBox getExtent(GDALDataset *dataset){
    double affine[6];
    dataset->GetGeoTransform(affine);
    return BoundingBox(geoLoc(0, dataset->GetRasterYSize(), affine), geoLoc(dataset->GetRasterXSize(), 0, affine));
}

int countSkirts(int nx, int ny){
    float neighbor_z = rasterData[ny * arr_width + nx];
    float current_z = p.z;
    int result = 0;

    if (current_z - neighbor_z > SkirtHeightThreshold.value && current_z - neighbor_z < SkirtHeightCap.value){
        while (current_z > neighbor_z){
            current_z -= SkirtIncrements.value;
            result++;
        }
        result++;
    }

    return result;
}

void writeSkirts(std::ofstream &f, int nx, int ny){
    float neighbor_z = rasterData[ny * arr_width + nx];
    float z_val = p.z;

    if (p.z - neighbor_z > SkirtHeightThreshold.value && p.z - neighbor_z < SkirtHeightCap.value){
        while (p.z > neighbor_z){
            p.z -= SkirtIncrements.value;
            f.write(reinterpret_cast<char*>(&p), psize);
        }
        p.z = neighbor_z;
        f.write(reinterpret_cast<char*>(&p), psize);
    }

    // Restore original p.z value
    p.z = z_val;
}

int main(int argc, char **argv) {
    cmdLineParse( argc-1 , &argv[1] , params );
    if( !InputFile.set || !OutputFile.set ) help(argv[0]);

    if (!SkirtHeightThreshold.set) SkirtHeightThreshold.value = 1.5f;
    if (!SkirtIncrements.set) SkirtIncrements.value = 0.1f;
    if (!SkirtHeightCap.set) SkirtHeightCap.value = 100.0f;

    logWriter.verbose = Verbose.set;
    // logWriter.outputFile = "odm_dem2points_log.txt";
    logArgs(params, logWriter);


    GDALDataset  *dataset;
    GDALAllRegister();
    dataset = (GDALDataset *) GDALOpen( InputFile.value, GA_ReadOnly );
    if( dataset != NULL )
    {
        arr_width = dataset->GetRasterXSize();
        arr_height = dataset->GetRasterYSize();

        logWriter("Raster Size is %dx%d\n", arr_width, arr_height);
        BoundingBox extent = getExtent(dataset);

        logWriter("Extent is (%f, %f), (%f, %f)\n", extent.min.x, extent.max.x, extent.min.y, extent.max.y);

        float ext_width = extent.max.x - extent.min.x;
        float ext_height = extent.max.y - extent.min.y;

        int vertex_count = (arr_height - 4) * (arr_width - 4);
        int skirt_points = 0;

        GDALRasterBand *band = dataset->GetRasterBand(1);

        rasterData = new float[arr_width * arr_height];

        if (band->RasterIO( GF_Read, 0, 0, arr_width, arr_height,
                            rasterData, arr_width, arr_height, GDT_Float32, 0, 0 ) == CE_Failure){
            std::cerr << "Cannot access raster data" << std::endl;
            exit(1);
        }

        // On the first pass we calculate the number of skirts
        // on the second pass we sample and write the points

        logWriter("Calculating skirts... ");

        for (int y = 2; y < arr_height - 2; y++){
            for (int x = 2; x < arr_width - 2; x++){
                p.z = rasterData[y * arr_width + x];

                skirt_points += countSkirts(x, y + 1);
                skirt_points += countSkirts(x, y - 1);
                skirt_points += countSkirts(x + 1, y);
                skirt_points += countSkirts(x - 1, y);
            }
        }

        logWriter("%d vertices will be added\n", skirt_points);
        logWriter("Total vertices: %d\n", (skirt_points + vertex_count));
        logWriter("Sampling and writing to file...");

        // Starting writing ply file
        std::ofstream f (OutputFile.value);
        f << "ply" << std::endl;

        if (IS_BIG_ENDIAN){
          f << "format binary_big_endian 1.0" << std::endl;
        }else{
          f << "format binary_little_endian 1.0" << std::endl;
        }

        f   << "element vertex " << (vertex_count + skirt_points) << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "property float nx" << std::endl
            << "property float ny" << std::endl
            << "property float nz" << std::endl
            << "end_header" << std::endl;

        // Normals are always pointing up
        p.nx = 0;
        p.ny = 0;
        p.nz = 1;

        for (int y = 2; y < arr_height - 2; y++){
            for (int x = 2; x < arr_width - 2; x++){
                p.z = rasterData[y * arr_width + x];
                p.x = extent.min.x + (static_cast<float>(x) / static_cast<float>(arr_width)) * ext_width;
                p.y = extent.max.y - (static_cast<float>(y) / static_cast<float>(arr_height)) * ext_height;

                f.write(reinterpret_cast<char*>(&p), psize);

                writeSkirts(f, x, y + 1);
                writeSkirts(f, x, y - 1);
                writeSkirts(f, x + 1, y);
                writeSkirts(f, x - 1, y);
            }
        }

        logWriter(" done!\n");

        f.close();
        GDALClose(dataset);

    }else{
        std::cerr << "Cannot open " << InputFile.value << std::endl;
    }
}
