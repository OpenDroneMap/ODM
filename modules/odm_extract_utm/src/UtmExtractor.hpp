#pragma once

// Logging
#include "Logger.hpp"

/*!
* \breif	The Coord struct		Class used in UtmExtractor to extract GPS positions from images and ODM output
*/
struct Coord
{
    double x, y, z;
    Coord(double ix, double iy, double iz) : x(ix), y(iy), z(iz) {}
};

class UtmExtractor
{
public:
    UtmExtractor();
    ~UtmExtractor();

	/*!
     * \brief   run   Runs the texturing functionality using the provided input arguments.
     *                For a list of the accepted arguments, please see the main page documentation or
     *                call the program with parameter "-help".
     * \param   argc  Application argument count.
     * \param   argv  Argument values.
     * \return  0     if successful.
     */
    int run (int argc, char **argv);

private:

	/*!
    * \brief parseArguments    Parses command line arguments.
    * \param argc              Application argument count.
    * \param argv              Argument values.
    */
    void parseArguments(int argc, char **argv);

	/*!
	* \breif extractUtm			Performs the extraction of coordinates inside the run function.
	*/
	void extractUtm();
	
   /*!
   * /brief Static method that converts a WGS84 longitude/latitude coordinate in decimal degrees to UTM.
   *
   * \param lon The longitude in decimal degrees (negative if western hemisphere).
   * \param lat The latitude in decimal degrees (negative if southern hemisphere).
   * \param alt The altitude in meters.
   * \param x Output parameter, the easting UTM value in meters.
   * \param y Output parameter, the northing UTM value in meters.
   * \param utmZone Input or output parameter, the UTM zone. Set to 99 for automatic selection.
   * \param hemisphere Input or output parameter, 'N' for norther hemisphere, 'S' for southern. Automatically selected if utmZone is 99.
   *
   * \returns True if successful (otherwise output parameters are 0)
   */
    static bool convert(const double &lon, const double &lat, const double &alt, double &x, double &y, double &z, int &utmZone, char &hemisphere);

	/*!
	* \brief Static method that parses a GPS position from jhead data.
	*
	* \param jheadDataStream Jhead data stream with EXIF information.
	* \param lon Output parameter, the longitude in decimal degrees.
	* \param lat Output parameter, the latitude in decimal degrees.
	* \param alt Output parameter, the altitude in meters.
	*
	* \returns True if successful (otherwise output parameters are 0)
	*/
    static bool parsePosition(std::ifstream &jheadStream, double &lon, double &lat, double &alt);
	
	/*!
     * \brief printHelp         Prints help, explaining usage. Can be shown by calling the program with arguments: "-help".
     */
    void printHelp();

	std::string imageListFileName_;		/**< Path to the image list. */
    std::string outputCoordFileName_;	/**< Path to the file to store the output textfile. */
    std::string imagesPath_;			/**< Path to the folder with all images in the image list. */

    Logger log_;                    /**< Logging object. */
    std::string logFile_;       /**< Path to store the log file. */

};

class UtmExtractorException : public std::exception
{
public:
    UtmExtractorException() : message("Error in OdmExtractUtm") {}
    UtmExtractorException(std::string msgInit) : message("Error in OdmExtractUtm:\n" + msgInit) {}
    ~UtmExtractorException() throw() {}
    virtual const char* what() const throw() {return message.c_str(); }

private:
    std::string message;        /**< The error message. */
};
