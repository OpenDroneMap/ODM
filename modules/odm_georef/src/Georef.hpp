#pragma once

// C++
#include <string>
#include <sstream>
#include <fstream>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
// Modified PCL
#include "modifiedPclFunctions.hpp"

// Logger
#include "Logger.hpp"

// Transformation
#include "FindTransform.hpp"

// PDAL matrix transform filter
#include "MatrixTransformFilter.hpp"

/*!
 * \brief   The GeorefSystem struct is used to store information about a georeference system.
 */
struct GeorefSystem
{
    std::string system_;        /**< The name of the system. **/
    double eastingOffset_;      /**< The easting offset for the georeference system. **/
    double northingOffset_;     /**< The northing offset for the georeference system. **/
    
    friend std::ostream& operator<<(std::ostream &os, const GeorefSystem &geo);
};

/*!
  * \brief  The GeorefGCP struct used to store information about a GCP.
  */
struct GeorefGCP
{
    double x_;              /**< The X coordinate of the GCP **/
    double y_;              /**< The Y coordinate of the GCP **/
    double z_;              /**< The Z coordinate of the GCP **/

    bool use_;              /**< Bool to check if the GCP is corresponding in the local model **/

    double localX_;         /**< The corresponding X coordinate in the model **/
    double localY_;         /**< The corresponding Y coordinate in the model **/
    double localZ_;         /**< The corresponding Z coordinate in the model **/

    size_t cameraIndex_;    /**< The index to the corresponding camera for the image. **/

    double pixelX_;            /**< The pixels x-position for the GCP in the corresponding image **/
    double pixelY_;            /**< The pixels y-position for the GCP in the corresponding image **/

    std::string image_;     /**< The corresponding image for the GCP **/
    std::string idgcp_;     /**< The corresponding identification for the GCP **/

    GeorefGCP();
    ~GeorefGCP();

    void extractGCP(std::istringstream &gcpStream);

    /*!
     * \brief getPos                Get the local position of the GCP.
     */
    Vec3 getPos();

    /*!
     * \brief getReferencedPos      Get the georeferenced position of the GCP.
     */
    Vec3 getReferencedPos();
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
     * \brief getPos     Get the local position of the camera.
     */
    Vec3 getPos();
    
    /*!
     * \brief getReferencedPos     Get the georeferenced position of the camera.
     */
    Vec3 getReferencedPos();

    /*!
     * \brief isValid     Whether this camera is valid based on its parameters.
     */
    bool isValid();
    
    double focalLength_;            /**< The focal length of the camera. */
    double k1_;                     /**< The k1 lens distortion parameter. **/
    double k2_;                     /**< The k2 lens distortion parameter. **/
    
    double easting_;                /**< The easting of the camera. **/
    double northing_;               /**< The northing of the camera. **/
    double altitude_;               /**< The altitude of the camera. **/
    
    Eigen::Affine3f* transform_;    /**< The rotation of the camera. **/
    Eigen::Vector3f* position_;     /**< The position of the camera. **/
    Eigen::Affine3f* pose_;         /**< The pose of the camera. **/
    
    friend std::ostream& operator<<(std::ostream &os, const GeorefCamera &cam);
};

/*!
 * \brief   The GeorefBestTriplet struct is used to store the best triplet found.
 */
struct GeorefBestTriplet
{
    size_t t_;          /**< First ordinate of the best triplet found. **/
    size_t s_;          /**< Second ordinate of the best triplet found. **/
    size_t p_;          /**< Third ordinate of the best triplet found. **/
    double err_;        /**< Error of this triplet. **/
};

/*!
 * \brief   The Georef class is used to transform a mesh into a georeferenced system.
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
     * \param   argc            Application argument count.
     * \param   argv            Argument values.
     */
    void parseArguments(int argc, char* argv[]);
    
    /*!
     * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
     */
    void printHelp();
    
    /*!
     * \brief setDefaultOutput     Setup the output file name given the input file name.
     */
    void setDefaultOutput();

    /*!
      * \brief setDefaultPointCloudOutput   Setup the output file name given the input file name.
      */
    void setDefaultPointCloudOutput();

    /*!
     * \brief createGeoreferencedModel    Makes the input file georeferenced and saves it to the output file.
     */
    void createGeoreferencedModel();

    /*!
     * \brief readCameras      Reads the camera information from the bundle file.
     */
    void readCameras();

    /*!
     * \brief readGCP           Reads the ground control points from the gcp file.
     */
    void readGCPs();

    /*!
      * \brief calculateGCPOffset       Calculates an offset weighted from the ground control points read in the readGCP function.
      */
    void calculateGCPOffset();

    /*!
      * \brief barycentricCoordinates   Returns the world position of a point inside a 2d triangle by using the triangle vertex positions.
      */
    pcl::PointXYZ barycentricCoordinates(pcl::PointXY point, pcl::PointXYZ vert0, pcl::PointXYZ vert1, pcl::PointXYZ vert2, pcl::PointXY p0, pcl::PointXY p1, pcl::PointXY p2);

    /*!
      * \brief performGeoreferencingWithGCP     Performs the georeferencing of the model with the ground control points.
      */
    void performGeoreferencingWithGCP();

    /*!
     * \brief createGeoreferencedModelFromGCPData    Makes the input file georeferenced and saves it to the output file.
     */
    void createGeoreferencedModelFromGCPData();

    /*!
     * \brief createGeoreferencedModelFromExifData    Makes the input file georeferenced and saves it to the output file.
     */
    void createGeoreferencedModelFromExifData();
    
    /*!
     *  \brief chooseBestGCPTriplet    Chooses the best triplet of GCPs to use when making the model georeferenced.
     */
    void chooseBestGCPTriplet(size_t &gcp0, size_t &gcp1, size_t &gcp2);

    /*!
     * \brief findBestGCPTriplet    Partitioned version of chooseBestGCPTriplet.
     */
    void findBestGCPTriplet(size_t &gcp0, size_t &gcp1, size_t &gcp2, size_t offset, size_t stride, double &minTotError);

    /*!
     * \brief chooseBestCameraTriplet    Chooses the best triplet of cameras to use when making the model georeferenced.
     */
    void chooseBestCameraTriplet(size_t &cam0, size_t &cam1, size_t &cam2);

    /*!
     * \brief findBestCameraTriplet    Partitioned version of chooseBestCameraTriplet.
     */
    void findBestCameraTriplet(size_t &cam0, size_t &cam1, size_t &cam2, size_t offset, size_t stride, double &minTotError);
    
    /*!
      * \brief printGeorefSystem        Prints a file containing information about the georeference system, next to the ouptut file.
      **/
    void printGeorefSystem();
    
    /*!
      * \brief printFinalTransform      Prints a file containing the final transform, next to the output file.
      **/
    template <typename Scalar>
    void printFinalTransform(const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

    
    /*!
      * \brief Loads a model from an .obj file (replacement for the pcl obj loader).
      *
      * \param inputFile Path to the .obj file.
      * \param mesh The model.
      * \return True if model was loaded successfully.
      */
    bool loadObjFile(std::string inputFile, pcl::TextureMesh &mesh);

    /*!
      * \brief Function is compied straight from the function in the pcl::io module.
      */
    bool readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                     Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                     int &file_version, int &data_type, unsigned int &data_idx,
                     const int offset);


    Logger          log_;                       /**< Logging object. */
    std::string     logFile_;                   /**< The path to the output log file. */
    
    std::string     finalTransformFile_;        /**< The path to the file for the final transform. */
    
    std::string     bundleFilename_;            /**< The path to the cameras bundle file. **/
    std::string     inputCoordFilename_;        /**< The path to the cameras exif gps positions file. **/
    std::string     outputCoordFilename_;       /**< The path to the cameras georeferenced gps positions file. **/
    std::string     gcpFilename_;               /**< The path to the GCP file **/
    std::string     transformFilename_;         /**< The path to the input transform file **/
    std::string     imagesListPath_;            /**< Path to the image list. **/
    std::string     imagesLocation_;            /**< The folder containing the images in the image list. **/
    std::string     inputObjFilename_;          /**< The path to the input mesh obj file. **/
    std::string     outputObjFilename_;         /**< The path to the output mesh obj file. **/
    std::string     inputPointCloudFilename_;   /**< The path to the input point cloud file. **/
    std::string     outputPointCloudFilename_;  /**< The path to the output point cloud file. **/
    std::string     georefFilename_;            /**< The path to the output offset file. **/
    std::string     outputPointCloudSrs_;                       /**< The spatial reference system of the point cloud file to be written. Can be an EPSG string (e.g. “EPSG:26910”) or a WKT string. **/

    bool            georeferencePointCloud_;
    bool            exportCoordinateFile_;
    bool            exportGeorefSystem_;
    bool            useGCP_;                    /**< Check if GCP-file is present and use this to georeference the model. **/
    bool            useTransform_;
    // double          bundleResizedTo_;           /**< The size used in the previous steps to calculate the camera focal_length. */

    std::vector<GeorefCamera> cameras_;         /**< A vector of all cameras. **/
    std::vector<GeorefGCP> gcps_;               /**< A vector of all GCPs. **/
    std::vector<std::string> imageList_;        /**< A vector containing the names of the corresponding cameras. **/
    
    GeorefSystem    georefSystem_;              /**< Contains the georeference system. **/

    bool            multiMaterial_;     /**< True if the mesh has multiple materials. **/

    std::vector<pcl::MTLReader> companions_; /**< Materials (used by loadOBJFile). **/
    void performFinalTransform(Mat4 &transMat, pcl::TextureMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &meshCloud, bool addUTM);
    
    template <typename Scalar>
    void transformPointCloud(const char *inputFile, const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform, const char *outputFile);
    
    void createGeoreferencedModelFromSFM();
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
