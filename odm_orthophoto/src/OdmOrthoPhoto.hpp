#pragma once

// C++
#include <limits.h>

// PCL
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

// OpenCV
#include <opencv2/core/core.hpp>

// Logger
#include "Logger.hpp"

/*!
 * \brief   The OdmOrthoPhoto class is used to create an orthograpic photo over a given area.
 *          The class reads an oriented textured mesh from an OBJ-file.
 *          The class uses file read from pcl.
 *          The class uses image read and write from opencv.
 */
class OdmOrthoPhoto
{
public:
    OdmOrthoPhoto();
    ~OdmOrthoPhoto();

    /*!
     * \brief   run     Runs the ortho photo functionality using the provided input arguments.
     *                  For a list of accepted arguments, pleas see the main page documentation or
     *                  call the program with parameter "-help".
     * \param   argc    Application argument count.
     * \param   argv    Argument values.
     * \return  0       if successful.
     */
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
      * \brief Create the ortho photo using the current settings.
      */
    void createOrthoPhoto();
    
    /*!
      * \brief Creates a transformation which aligns the area for the orthophoto.
      */
    Eigen::Transform<float, 3, Eigen::Affine> getROITransform(float xMin, float yMin) const;
    
    /*!
      * \brief Renders a triangle into the ortho photo.
      * \param texture The texture of the polygon.
      * \param polygon The polygon as athree indices relative meshCloud.
      * \param meshCloud Contains all vertices.
      * \param uvs Contains the texture coordiantes for the active material.
      * \param faceIndex The index of the face.
      */
    void drawTexturedTriangle(const cv::Mat &texture, const pcl::Vertices &polygon, const pcl::PointCloud<pcl::PointXYZ>::Ptr &meshCloud, const std::vector<Eigen::Vector2f> &uvs, size_t faceIndex);
    
    /*!
      * \brief Calcualtes the barycentric coordinates of a point in a triangle.
      * \param v1 The first triangle vertex.
      * \param v2 The second triangle vertex.
      * \param v3 The third triangle vertex.
      * \param x The x coordinate of the point.
      * \param y The y coordinate of the point.
      * \param l1 The first vertex weight.
      * \param l2 The second vertex weight.
      * \param l3 The third vertex weight.
      */
    void getBarycentricCoordiantes(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3, float x, float y, float &l1, float &l2, float &l3) const;
    
    /*!
      * \brief Check if a given polygon is a sliver polygon.
      * \param v1 The first vertex of the polygon.
      * \param v2 The second vertex of the polygon.
      * \param v3 The third vertex of the polygon.
      */
    bool isSliverPolygon(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3) const;
    
    /*!
      * \brief Check if the model is suitable for ortho photo generation.
      * \param mesh The
      */
    bool isModelOk(const pcl::TextureMesh &mesh);
    
    Logger          log_;               /**< Logging object. */

    std::string     inputFile_;         /**< Path to the textured mesh as an obj-file. */
    std::string     outputFile_;        /**< Path to the destination file. */
    std::string     logFile_;           /**< Path to the log file. */
  
    float           resolution_;        /**< The number of pixels per meter in the ortho photo. */
    
    Eigen::Vector2f boundryPoint1_;     /**< The first boundry point for the ortho photo. */
    Eigen::Vector2f boundryPoint2_;     /**< The second boundry point for the ortho photo. */
    Eigen::Vector2f boundryPoint3_;     /**< The third boundry point for the ortho photo. */
    Eigen::Vector2f boundryPoint4_;     /**< The fourth boundry point for the ortho photo. */
    
    cv::Mat         photo_;             /**< The ortho photo as an OpenCV matrix, CV_8UC3. */
    cv::Mat         depth_;             /**< The depth of the ortho photo as an OpenCV matrix, CV_32F. */
    
    bool            multiMaterial_;     /**< True if the mesh has multiple materials. **/
};

/*!
 * \brief The OdmOrthoPhoto class
 */
class OdmOrthoPhotoException : public std::exception
{

public:
    OdmOrthoPhotoException() : message("Error in OdmOrthoPhoto") {}
    OdmOrthoPhotoException(std::string msgInit) : message("Error in OdmOrthoPhoto:\n" + msgInit) {}
    ~OdmOrthoPhotoException() throw() {}
    virtual const char* what() const throw() {return message.c_str(); }

private:
    std::string message;    /**< The error message **/
};
