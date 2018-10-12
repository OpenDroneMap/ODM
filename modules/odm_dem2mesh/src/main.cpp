#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
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
        arr_width = dataset->GetRasterXSize();
        arr_height = dataset->GetRasterYSize();

        logWriter("Raster Size is %dx%d\n", arr_width, arr_height);
        BoundingBox extent = getExtent(dataset);

        logWriter("Extent is (%f, %f), (%f, %f)\n", extent.min.x, extent.max.x, extent.min.y, extent.max.y);

        float ext_width = extent.max.x - extent.min.x;
        float ext_height = extent.max.y - extent.min.y;

        unsigned long long int vertex_count = static_cast<unsigned long long int>(arr_height) *
                                              static_cast<unsigned long long int>(arr_width);

        logWriter("Reading raster...\n");

        // If the DSM is really large, we only sample a subset of it
        // to remain within INT_MAX vertices. This does not happen often,
        // but it's a safeguard to make sure we'll get an output and not
        // overflow.
        int stride = 8;
        while (vertex_count > INT_MAX){
            stride *= 2;
            vertex_count = static_cast<int>(std::ceil((arr_height / static_cast<double>(stride))) *
                                            std::ceil((arr_width / static_cast<double>(stride))));
        }

        if (stride != 1){
            logWriter("Warning: DSM is large, will sample using stride value of %d\n", stride);
        }
        logWriter("Total vertices before simplification: %llu\n", vertex_count);

        GDALRasterBand *band = dataset->GetRasterBand(1);

        int subdivisions = 1;
        int numBlocks = (int)pow(2, subdivisions == 1 ? 0 : subdivisions);
        int blockSizeX = arr_width / subdivisions;
        int blockSizeY = arr_height / subdivisions;
        int blockXPad = 0; // Blocks > 0 need to re-add a column for seamless meshing
        int blockYPad = 0; // Blocks > 0 need to re-add a row for seamless meshing

        logWriter("Splitting area in %d\n", numBlocks);
        logWriter("Block size is %d, %d\n", blockSizeX, blockSizeY);

        rasterData = new float[blockSizeX + stride * 8];

        for (int blockX = 0; blockX < subdivisions; blockX++){
            int xOffset = blockX * blockSizeX - blockXPad;

            for (int blockY = 0; blockY < subdivisions; blockY++){
                int yOffset = blockY * blockSizeY - blockYPad;

                Simplify::vertices.clear();
                Simplify::triangles.clear();

                logWriter("Processing block (%d,%d)\n", blockX, blockY);

                for (int y = 0; y < blockSizeY + blockYPad; y += stride){
                    if (band->RasterIO( GF_Read, xOffset, yOffset + y, blockSizeX + blockXPad, 1,
                                        rasterData, blockSizeX + blockXPad, 1, GDT_Float32, 0, 0 ) == CE_Failure){
                        std::cerr << "Cannot access raster data" << std::endl;
                        exit(1);
                    }

                    for (int x = 0; x < blockSizeX + blockXPad; x += stride){
                        Simplify::Vertex v;
                        v.p.x = extent.min.x + (static_cast<float>(xOffset + x) / static_cast<float>(arr_width)) * ext_width;
                        v.p.y = extent.max.y - (static_cast<float>(yOffset + y) / static_cast<float>(arr_height)) * ext_height;
                        v.p.z = rasterData[x];

                        Simplify::vertices.push_back(v);
                    }
                }

                unsigned int cols = static_cast<unsigned int>(std::ceil(((blockSizeX + blockXPad) / static_cast<double>(stride))));
                unsigned int rows = static_cast<unsigned int>(std::ceil(((blockSizeY + blockYPad) / static_cast<double>(stride))));

                for (unsigned int y = 0; y < rows - 1; y++){
                    for (unsigned int x = 0; x < cols - 1; x++){
                        Simplify::Triangle t1;
                        t1.v[0] = cols * (y + 1) + x;
                        t1.v[1] = cols * y + x + 1;
                        t1.v[2] = cols * y + x;

                        Simplify::triangles.push_back(t1);

                        Simplify::Triangle t2;
                        t2.v[0] = cols * (y + 1) + x;
                        t2.v[1] = cols * (y + 1) + x + 1;
                        t2.v[2] = cols * y + x + 1;

                        Simplify::triangles.push_back(t2);
                    }
                }

                double agressiveness = 6.0;
                double max_threshold = 0.30;
                int target_count = std::min((MaxVertexCount.value * 2) / numBlocks, static_cast<int>(Simplify::triangles.size()));

                logWriter("Sampled %d faces, target is %d\n", static_cast<int>(Simplify::triangles.size()), target_count);
                logWriter("Simplifying...\n");

                unsigned long start_size = Simplify::triangles.size();
                Simplify::simplify_mesh(target_count, agressiveness, Verbose.set, max_threshold);
                if ( Simplify::triangles.size() >= start_size) {
                    std::cerr << "Unable to reduce mesh.\n";
                    exit(1);
                }

                logWriter("Writing to file...");

                // Start writing ply file
                std::stringstream ss;
                ss << OutputFile.value << "." << blockX << "-" << blockY << ".ply";
                std::ofstream f (ss.str());
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

                blockYPad = stride * 8;
            }

            blockYPad = 0;
            blockXPad = stride * 8;
        }

        delete[] rasterData;
        GDALClose(dataset);
    }else{
        std::cerr << "Cannot open " << InputFile.value << std::endl;
    }
}
