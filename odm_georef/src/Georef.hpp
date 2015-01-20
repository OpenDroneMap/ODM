#pragma once

// C++
#include <string>
#include <sstream>
#include <fstream>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

// Logger
#include "Logger.hpp"

// Transformation
#include "FindTransform.hpp"

/*!
 * \brief   The GeorefSystem struct is used to store information about a georeference system.
 */
struct GeorefSystem
{
    std::string system_;        /**< The name of the system. **/
    double falseEasting_;       /**< The false easting of the cameras. **/
    double falseNorthing_;      /**< The false northing of the cameras. **/
    
    friend std::ostream& operator<<(std::ostream &os, const GeorefSystem &geo);
};

/*!
 * \brief   The GeorefCamera struct is used to store information about a camera.
 */
struct GeorefCamera
{
    GeorefCamera();
    GeorefCamera(const GeorefCamera &other);
    ~GeorefCamera();
    
    /*!
     * \brief extractCamera     Extracts a camera's intrinsic and extrinsic parameters from a stream.
     */
    void extractCamera(std::ifstream &bundleStream);
    
    /*!
     * \brief extractCameraGeoref     Extracts a camera's world position from a stream.
     */
    void extractCameraGeoref(std::istringstream &coordStream);
    
    /*!
     * \brief getReferencedPos     Get the local position of the camera.
     */
    Vec3 getPos();
    
    /*!
     * \brief getReferencedPos     Get the georeferenced position of the camera.
     */
    Vec3 getReferencedPos();
    
    double focalLength_;            /**< The focal length of the camera. */
    double k1_;                     /**< The k1 lens distortion parameter. **/
    double k2_;                     /**< The k2 lens distortion parameter. **/
    
    double easting_;                /**< The easting of the camera. **/
    double northing_;               /**< The northing of the camera. **/
    double altitude_;               /**< The altitude of the camera. **/
    
    Eigen::Affine3f* transform_;    /**< The rotation of the camera. **/
    Eigen::Vector3f* position_;     /**< The position of the camera. **/
    
    friend std::ostream& operator<<(std::ostream &os, const GeorefCamera &cam);
};

/*!
 * \brief   The OdmOrthoPhoto class is used to transform a mesh into a georeferenced system.
 *          The class reads camera positions from a bundle file.
 *          The class reads the georefenced camera positions from a coords file.
 *          The class reads a textured mesh from an OBJ-file.
 *          The class writes the georeferenced textured mesh to an OBJ-file.
 *          The class uses file read and write from pcl.
 */
class Georef
{
public:
    Georef();
    ~Georef();
    
    int run(int argc, char* argv[]);
    
private:
    
    /*!
     * \brief parseArguments    Parses command line arguments.
     * \param   argc    Application argument count.
     * \param   argv    Argument values.
     */
    void parseArguments(int argc, char* argv[]);
    
    /*!
     * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
     */
    void printHelp();
    
    /*!
     * \brief makeDefaultOutput     Setup the output file name given the input file name.
     */
    void makeDefaultOutput();
    
    /*!
     * \brief makeGeoreferencedModel    Makes the input file georeferenced and saves it to the output file.
     */
    void makeGeoreferencedModel();
    
    /*!
     * \brief chooseBestCameraTriplet    Chooses the best triplet of cameras to use when makin gthe model georeferenced.
     */
    void chooseBestCameraTriplet(size_t &cam0, size_t &cam1, size_t &cam2);
    
    /*!
      * \brief printGeorefSystem        Prints a file containing information about the georeference system, next to the ouptut file.
      **/
    void printGeorefSystem();
    
    Logger          log_;               /**< Logging object. */
    std::string     logFile_;           /**< The path to the output log file. */
    
    std::string     bundleFilename_;    /**< The path to the cameras bundle file. **/
    std::string     coordFilename_;     /**< The path to the cameras georeference file. **/
    std::string     inputObjFilename_;  /**< The path to the input mesh obj file. **/
    std::string     outputObjFilename_; /**< The path to the output mesh obj file. **/
    
    std::vector<GeorefCamera> cameras_; /**< A vector of all cameras. **/
    
    GeorefSystem    georefSystem_;  /**< Contains the georeference system. **/
};

/*!
 * \brief The Georef class
 */
class GeorefException : public std::exception
{

public:
    GeorefException() : message("Error in Georef") {}
    GeorefException(std::string msgInit) : message("Error in Georef:\n" + msgInit) {}
    ~GeorefException() throw() {}
    virtual const char* what() const throw() {return message.c_str(); }

private:
    std::string message;    /**< The error message **/
};
