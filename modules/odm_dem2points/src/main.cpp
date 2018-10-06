#include <iostream>
#include <string>
#include <fstream>
#include "CmdLineParser.h"
#include "Logger.h"
#include "Simplify.h"

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
} p;
size_t psize = sizeof(float) * 3;

struct PlyFace{
    uint32_t p1;
    uint32_t p2;
    uint32_t p3;
} face;
size_t fsize = sizeof(uint32_t) * 3;

float *rasterData;
int arr_width, arr_height;

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

cmdLineParameter< char* >
    InputFile( "inputFile" ) ,
    OutputFile( "outputFile" );
cmdLineParameter< int >
    MaxFaceCount( "maxFaceCount" );
cmdLineReadable
	Verbose( "verbose" );

cmdLineReadable* params[] = {
    &InputFile , &OutputFile , &Verbose ,
    NULL
};

void help(char *ex){
    std::cout << "Usage: " << ex << std::endl
              << "\t -" << InputFile.name << " <input DSM raster>" << std::endl
              << "\t -" << OutputFile.name << " <output PLY mesh>" << std::endl
              << "\t [-" << MaxFaceCount.name << " <target number faces> (Default: 100000)]" << std::endl
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

int main(int argc, char **argv) {
    cmdLineParse( argc-1 , &argv[1] , params );
    if( !InputFile.set || !OutputFile.set ) help(argv[0]);
    if ( !MaxFaceCount.set ) MaxFaceCount.value = 100000;

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

        int vertex_count = (arr_height - 2) * (arr_width - 2);

        GDALRasterBand *band = dataset->GetRasterBand(1);

        rasterData = new float[arr_width * arr_height];

        if (band->RasterIO( GF_Read, 0, 0, arr_width, arr_height,
                            rasterData, arr_width, arr_height, GDT_Float32, 0, 0 ) == CE_Failure){
            std::cerr << "Cannot access raster data" << std::endl;
            exit(1);
        }

        logWriter("Total vertices before simplification: %d\n", vertex_count);
        logWriter("Sampling...\n");

        for (int y = 1; y < arr_height - 1; y++){
            for (int x = 1; x < arr_width - 1; x++){
                Simplify::Vertex v;
                v.p.x = extent.min.x + (static_cast<float>(x) / static_cast<float>(arr_width)) * ext_width;
                v.p.y = extent.max.y - (static_cast<float>(y) / static_cast<float>(arr_height)) * ext_height;
                v.p.z = rasterData[y * arr_width + x];

                Simplify::vertices.push_back(v);
            }
        }

        unsigned int cols = arr_width - 2;
        unsigned int rows = arr_height - 2;

        for (unsigned int y = 0; y < rows - 1; y++){
            for (unsigned int x = 0; x < cols - 1; x++){
                Simplify::Triangle t1;
                t1.v[0] = cols * (y + 1) + x;
                t1.v[1] = cols * y + x + 1;
                t1.v[2] = cols * y + x;
                t1.attr = 0;
                t1.material = -1;

                Simplify::triangles.push_back(t1);

                Simplify::Triangle t2;
                t2.v[0] = cols * (y + 1) + x;
                t2.v[1] = cols * (y + 1) + x + 1;
                t2.v[2] = cols * y + x + 1;
                t2.attr = 0;
                t2.material = -1;

                Simplify::triangles.push_back(t2);
            }
        }

        double agressiveness = 7.0;
        int target_count = std::min(MaxFaceCount.value, static_cast<int>(Simplify::triangles.size()));

        logWriter("Sampled %d faces, target is %d\n", static_cast<int>(Simplify::triangles.size()), MaxFaceCount.value);
        logWriter("Simplifying...\n");

        unsigned long start_size = Simplify::triangles.size();
        Simplify::simplify_mesh(target_count, agressiveness, true);
        if ( Simplify::triangles.size() >= start_size) {
            std::cerr << "Unable to reduce mesh.\n";
            exit(1);
        }

        logWriter("Writing to file...");

        // Start writing ply file
        std::ofstream f (OutputFile.value);
        f << "ply" << std::endl;

        if (IS_BIG_ENDIAN){
          f << "format binary_big_endian 1.0" << std::endl;
        }else{
          f << "format binary_little_endian 1.0" << std::endl;
        }

        f   << "element vertex " << Simplify::vertices.size() << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "element face " << Simplify::triangles.size() << std::endl
            << "property list uint8 uint32 vertex_indices" << std::endl
            << "end_header" << std::endl;

        for(Simplify::Vertex &v : Simplify::vertices){
            p.x = static_cast<float>(v.p.x);
            p.y = static_cast<float>(v.p.y);
            p.z = static_cast<float>(v.p.z);
            f.write(reinterpret_cast<char *>(&p), psize);
        }

        uint8_t three = 3;
        for(Simplify::Triangle &t : Simplify::triangles){
            face.p1 = static_cast<uint32_t>(t.v[0]);
            face.p2 = static_cast<uint32_t>(t.v[1]);
            face.p3 = static_cast<uint32_t>(t.v[2]);

            f.write(reinterpret_cast<char *>(&three), sizeof(three));
            f.write(reinterpret_cast<char *>(&face), fsize);
        }

        logWriter(" done!\n");

        f.close();
        GDALClose(dataset);
    }else{
        std::cerr << "Cannot open " << InputFile.value << std::endl;
    }
}
