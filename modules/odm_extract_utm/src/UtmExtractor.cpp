// STL
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <ctype.h>
#include <sstream>
#include <math.h>

// Proj4
#include <proj_api.h>

// This
#include "UtmExtractor.hpp"

UtmExtractor::UtmExtractor() : log_(false)
{
    logFile_ = "odm_extracting_utm_log.txt";
}

UtmExtractor::~UtmExtractor()
{
}



int UtmExtractor::run(int argc, char **argv)
{
	if (argc <= 1)
	{
		printHelp();
		return EXIT_SUCCESS;
	}
	
	try
	{
		parseArguments(argc, argv);
		extractUtm();
	}
	catch (const UtmExtractorException& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << e.what() << "\n";
        log_.printToFile(logFile_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Error in OdmExtractUtm:\n";
        log_ << e.what() << "\n";
        log_.printToFile(logFile_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }
    catch (...)
    {
        log_.setIsPrintingInCout(true);
        log_ << "Unknown error in OdmExtractUtm:\n";
        log_.printToFile(logFile_);
        log_ << "For more detailed information, see log file." << "\n";
        return EXIT_FAILURE;
    }

    log_.printToFile(logFile_);
    return EXIT_SUCCESS;
}

void UtmExtractor::parseArguments(int argc, char **argv)
{
	for(int argIndex = 1; argIndex < argc; ++argIndex)
	{
		// The argument to be parsed
		std::string argument = std::string(argv[argIndex]);
		if (argument == "-help")
		{
			printHelp();
		}
		else if (argument == "-verbose")
		{
			log_.setIsPrintingInCout(true);
		}
		else if (argument == "-imageListFile")
		{
			++argIndex;
			if (argIndex >= argc)
			{
				throw UtmExtractorException("Missing argument for '" + argument + "'.");
			}
			imageListFileName_ = std::string(argv[argIndex]);
			std::ifstream testFile(imageListFileName_.c_str(), std::ios_base::binary);
			if (!testFile.is_open())
			{
				throw UtmExtractorException("Argument '" + argument + "' has a bad value (file not accessible).");
			}
            log_ << "imageListFile was set to: " << imageListFileName_ << "\n";
		}
		else if (argument == "-imagesPath")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw UtmExtractorException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> imagesPath_;
            if (ss.bad())
            {
                throw UtmExtractorException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "imagesPath was set to: " << imagesPath_ << "\n";
        }
		else if (argument == "-outputCoordFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw UtmExtractorException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> outputCoordFileName_;
            if (ss.bad())
            {
                throw UtmExtractorException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "outputCoordFile was set to: " << outputCoordFileName_ << "\n";
        }
        else if (argument == "-logFile")
        {
            ++argIndex;
            if (argIndex >= argc)
            {
                throw UtmExtractorException("Missing argument for '" + argument + "'.");
            }
            std::stringstream ss(argv[argIndex]);
            ss >> logFile_;
            if (ss.bad())
            {
                throw UtmExtractorException("Argument '" + argument + "' has a bad value. (wrong type)");
            }
            log_ << "logFile_ was set to: " << logFile_ << "\n";
        }
		else
        {
            printHelp();
            throw UtmExtractorException("Unrecognised argument '" + argument + "'.");
        }
    }

}

void UtmExtractor::extractUtm()
{
	// Open file listing all used camera images
  std::ifstream imageListStream(imageListFileName_.c_str());
  if (!imageListStream.good()) {
    throw UtmExtractorException("Failed to open " + imageListFileName_ + " for reading.");
  }

  // Traverse images
  int utmZone = 99; // for auto-select
  char hemisphere;
  std::string imageFilename;
  std::vector<Coord> coords;
  while (getline(imageListStream, imageFilename)) {
    // Run jhead on image to extract EXIF data to temporary file
    std::string commandLine = "jhead -v " + imagesPath_ + "/" + imageFilename + " > extract_utm_output.txt";
    system(commandLine.c_str());
    
    // Read temporary EXIF data file
    std::ifstream jheadDataStream;
    jheadDataStream.open("extract_utm_output.txt");
    if (!jheadDataStream.good()) {
      throw UtmExtractorException("Failed to open temporary jhead data file extract_utm_output.txt");
    }
    
    // Delete temporary file
    remove("extract_utm_output.txt");

    // Parse jhead output
    double lon, lat, alt;
    if (!parsePosition(jheadDataStream, lon, lat, alt)) {
      throw UtmExtractorException("Failed parsing GPS position.");
      jheadDataStream.close();
    }
    jheadDataStream.close();

    // Convert to UTM
    double x, y, z;
    convert(lon, lat, alt, x, y, z, utmZone, hemisphere);
    coords.push_back(Coord(x, y, z));
  }
  imageListStream.close();

  // Calculate average
  double dx = 0.0, dy = 0.0;
  double num = static_cast<double>(coords.size());
  for (std::vector<Coord>::iterator iter = coords.begin(); iter != coords.end(); ++iter) {
    dx += iter->x/num;
    dy += iter->y/num;
  }

  dx = floor(dx);
  dy = floor(dy);

  // Open output file
  std::ofstream outputCoordStream(outputCoordFileName_.c_str());
  if (!outputCoordStream.good()) {
    throw UtmExtractorException("Failed to openg " + outputCoordFileName_ + " for writing.");
  }
  outputCoordStream.precision(10);

  // Write coordinate file
  outputCoordStream << "WGS84 UTM " << utmZone << hemisphere << std::endl;
  outputCoordStream << dx << " " << dy << std::endl;
  for (std::vector<Coord>::iterator iter = coords.begin(); iter != coords.end(); ++iter) {
    outputCoordStream << (iter->x - dx) << " " << (iter->y - dy) << " " << iter->z << std::endl;
  }

  outputCoordStream.close();
}

bool UtmExtractor::convert(const double &lon, const double &lat, const double &alt, double &x, double &y, double &z, int &utmZone, char &hemisphere)
{
  x = y = z = 0.0;

  // Create WGS84 longitude/latitude coordinate system
  projPJ pjLatLon = pj_init_plus("+proj=latlong +datum=WGS84");
  if (!pjLatLon) {
    throw UtmExtractorException("Couldn't create WGS84 coordinate system with PROJ.4.");
    return false;
  }

  // Calculate UTM zone if it's set to magic 99
  // NOTE: Special UTM cases in Norway/Svalbard not supported here
  if (utmZone == 99) {
    utmZone = ((static_cast<int>(floor((lon + 180.0)/6.0)) % 60) + 1);
    if (lat < 0)
      hemisphere = 'S';
    else
      hemisphere = 'N';
  }

  std::ostringstream ostr;
  ostr << utmZone;
  if (hemisphere == 'S')
    ostr << " +south";

  // Create UTM coordinate system
  projPJ pjUtm = pj_init_plus(("+proj=utm +datum=WGS84 +zone=" + ostr.str()).c_str());
  if (!pjUtm) {
    throw UtmExtractorException("Couldn't create UTM coordinate system with PROJ.4.");
    return false;
  }

  // Convert to radians
  x = lon * DEG_TO_RAD;
  y = lat * DEG_TO_RAD;
  z = alt;

  // Transform
  int res = pj_transform(pjLatLon, pjUtm, 1, 1, &x, &y, &z);
  if (res != 0) {
    throw UtmExtractorException("Failed to transform coordinates");
    return false;
  }

  return true;
}

bool UtmExtractor::parsePosition(std::ifstream &jheadStream, double &lon, double &lat, double &alt)
{
  lon = lat = alt = 0.0;

  // Parse position
  std::string str;
  std::string latStr, lonStr, altStr;
  while (std::getline(jheadStream, str))
  {
    size_t index = str.find("GPS Latitude : ");
    if (index != std::string::npos)
    {
        latStr = str.substr(index + 15);
        size_t find = latStr.find_first_of("0123456789");
        if(std::string::npos == find)
        {
            throw UtmExtractorException("Image is missing GPS Latitude data");
        }

    }
    index = str.find("GPS Longitude: ");
    if (index != std::string::npos)
    {
        lonStr = str.substr(index + 15);
        size_t find = lonStr.find_first_of("0123456789");
        if(std::string::npos == find)
        {
            throw UtmExtractorException("Image is missing GPS Latitude data");
        }
    }
    index = str.find("GPSAltitude");
    if (index != std::string::npos)
    {
       altStr = str.substr(index + 12);
       size_t find = altStr.find_first_of("0123456789");
       if(std::string::npos == find)
       {
           throw UtmExtractorException("Image is missing GPS Latitude data");
       }
    }
  }

  if (lonStr.empty() || latStr.empty()) {
    throw UtmExtractorException("No valid GPS position found");
    return false;
  }

  // Parse longitude
  std::string hemisphere;
  double degrees, minutes, seconds;
  std::istringstream istr(lonStr);
  char degChar = 'd', minChar = 'm', secChar = 's';
  istr >> hemisphere >> degrees >> degChar >> minutes >> minChar >> seconds >> secChar;
  lon = (hemisphere == "W" ? -1 : 1) * (degrees + minutes/60.0 + seconds/3600.0);

  // Parse latitude
  istr.clear();
  istr.str(latStr);
  istr >> hemisphere >> degrees >> degChar >> minutes >> minChar >> seconds >> secChar;
  lat = (hemisphere == "S" ? -1 : 1) * (degrees + minutes/60.0 + seconds/3600.0);

  if (!altStr.empty())
  {
      size_t index = altStr.find_last_of("=");
      if (index != std::string::npos)
      {
          altStr = altStr.substr(index + 1);
          istr.clear();
          istr.str(altStr);
          
          char dummyChar;
          int nominator, denominator;
          istr >> nominator;
          istr >> dummyChar;
          istr >> denominator;
          
          alt = static_cast<double>(nominator)/static_cast<double>(denominator);
      }
  }

  return true;
}

void UtmExtractor::printHelp()
{
log_.setIsPrintingInCout(true);

    log_ << "Purpose:\n";
    log_ << "Create a coordinate file containing the GPS positions of all cameras to be used later in the ODM toolchain for automatic georeferecing.\n";

    log_ << "Usage:\n";
    log_ << "The program requires paths to a image list file, a image folder path and an output textfile to store the results.\n";

    log_ << "The following flags are available:\n";
    log_ << "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
    log_ << "Call the program with flag \"-verbose\", to print log messages in the standard output.\n\n";

    log_ << "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configurable:\n";
    log_ << "\"-imageListFile <path>\" (mandatory)\n";
    log_ << "Path to the list containing the image names used in the bundle.out file.\n";

    log_ << "\"-imagesPath <path>\" (mandatory)\n";
    log_ << "Path folder containing all images in the imageListFile.\n";

    log_ << "\"-outputCoordFile <path>\" (mandatory)\n";
    log_ << "Path output textfile.\n";

    log_.setIsPrintingInCout(false);
}
