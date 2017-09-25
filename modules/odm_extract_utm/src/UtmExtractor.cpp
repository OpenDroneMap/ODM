// STL
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <ctype.h>
#include <sstream>
#include <math.h>
#include <exiv2/exiv2.hpp>

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

    // Read image and load metadata
    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(imagesPath_ + "/" + imageFilename);
    if (image.get() == 0) {
      std::string error(imageFilename);
      error += ": Image cannot be read";
      throw Exiv2::Error(1, error);
    }
    else {
      image->readMetadata();

      Exiv2::ExifData &exifData = image->exifData();
      if (exifData.empty()) {
        std::string error(imageFilename);
        error += ": No Exif data found in the file";
        throw Exiv2::Error(1, error);
      }

      // Parse exif data for positional data
      double lon, lat, alt = 0.0;

      parsePosition(exifData, lon, lat, alt);

      if (lon == 0.0 || lat == 0.0 || alt == 0.0) {
        std::string error("Failed parsing GPS position for " + imageFilename);
        throw UtmExtractorException(error);
      }
      // Convert to UTM
      double x, y, z = 0.0;
      convert(lon, lat, alt, x, y, z, utmZone, hemisphere);
      if (x == 0.0 || y == 0.0 || z == 0.0) {
        std::string error("Failed to convert GPS position to UTM for " + imageFilename);
        throw UtmExtractorException(error);
      }
      coords.push_back(Coord(x, y, z));
    }
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
    throw UtmExtractorException("Failed to open " + outputCoordFileName_ + " for writing.");
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

void UtmExtractor::convert(const double &lon, const double &lat, const double &alt, double &x, double &y, double &z, int &utmZone, char &hemisphere)
{
  // Create WGS84 longitude/latitude coordinate system
  projPJ pjLatLon = pj_init_plus("+proj=latlong +datum=WGS84");
  if (!pjLatLon) {
    throw UtmExtractorException("Couldn't create WGS84 coordinate system with PROJ.4.");
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
  }

  // Convert to radians
  x = lon * DEG_TO_RAD;
  y = lat * DEG_TO_RAD;
  z = alt;

  // Transform
  int res = pj_transform(pjLatLon, pjUtm, 1, 1, &x, &y, &z);
  if (res != 0) {
    throw UtmExtractorException("Failed to transform coordinates");
  }
}

void UtmExtractor::parsePosition(Exiv2::ExifData &exifData, double &lon, double &lat, double &alt)
{
  Exiv2::Exifdatum& latitudeTag = exifData["Exif.GPSInfo.GPSLatitude"];
  Exiv2::Exifdatum& latitudeRef = exifData["Exif.GPSInfo.GPSLatitudeRef"];
  Exiv2::Exifdatum& longitudeTag = exifData["Exif.GPSInfo.GPSLongitude"];
  Exiv2::Exifdatum& longitudeRef = exifData["Exif.GPSInfo.GPSLongitudeRef"];
  Exiv2::Exifdatum& altitudeTag = exifData["Exif.GPSInfo.GPSAltitude"];
  Exiv2::Exifdatum& altitudeRef = exifData["Exif.GPSInfo.GPSAltitudeRef"];

  // Latitude: parse into a double
  if (latitudeTag.count() < 3)
    throw UtmExtractorException("Image is missing GPS Latitude data");
  else {
    Exiv2::URational rLat[] = {latitudeTag.toRational(0), latitudeTag.toRational(1), latitudeTag.toRational(2)};
    bool south = (strcmp(latitudeRef.toString().c_str(), "S") == 0);
    double degrees, minutes, seconds;

    degrees = (double)rLat[0].first / (double)rLat[0].second;
    minutes = (double)rLat[1].first / (double)rLat[1].second / 60.0;
    seconds = (double)rLat[2].first / (double)rLat[2].second / 3600.0;
    lat = (south ? -1 : 1) * (degrees + minutes + seconds);
  }

  // Longitude
  if (longitudeTag.count() < 3)
    throw UtmExtractorException("Image is missing GPS Longitude data");
  else {
    Exiv2::URational rLon[] = {longitudeTag.toRational(0), longitudeTag.toRational(1), longitudeTag.toRational(2)};
    bool west = (strcmp(longitudeRef.toString().c_str(), "W") == 0);
    double degrees, minutes, seconds;

    degrees = (double)rLon[0].first / (double)rLon[0].second;
    minutes = (double)rLon[1].first / (double)rLon[1].second / 60.0;
    seconds = (double)rLon[2].first / (double)rLon[2].second / 3600.0;
    lon = (west ? -1 : 1) * (degrees + minutes + seconds);
  }

  // Altitude
  if (altitudeTag.count() < 1)
    throw UtmExtractorException("Image is missing GPS Altitude data");
  else {
    Exiv2::URational rAlt = altitudeTag.toRational(0);
    bool below = (altitudeRef.count() >= 1 && altitudeRef.toLong() == 1);
    alt = (below ? -1 : 1) * (double) rAlt.first / (double) rAlt.second;
  }
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
