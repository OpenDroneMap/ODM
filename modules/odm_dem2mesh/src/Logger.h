#include <cstdio>
#include <cstdarg>
#include "CmdLineParser.h"

struct Logger{
    bool verbose;
    const char* outputFile;

    Logger(){
        this->verbose = false;
        this->outputFile = NULL;
    }

    void operator() ( const char* format , ... )
    {
            if( outputFile )
            {
                    FILE* fp = fopen( outputFile , "a" );
                    va_list args;
                    va_start( args , format );
                    vfprintf( fp , format , args );
                    fclose( fp );
                    va_end( args );
            }
            if( verbose )
            {
                    va_list args;
                    va_start( args , format );
                    vprintf( format , args );
                    va_end( args );
            }
    }
};
