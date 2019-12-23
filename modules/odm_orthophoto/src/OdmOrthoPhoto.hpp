#pragma once

// C++
#include <limits.h>
#include <istream>
#include <ostream>

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

// GDAL
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()

// Logger
#include "Logger.hpp"

struct Bounds{
    float xMin;
    float xMax;
    float yMin;
    float yMax;

    Bounds() : xMin(0), xMax(0), yMin(0), yMax(0) {}
    Bounds(float xMin, float xMax, float yMin, float yMax) :
        xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax){}
    Bounds(const Bounds &b) {
        xMin = b.xMin;
        xMax = b.xMax;
        yMin = b.yMin;
        yMax = b.yMax;
    }
};

/*!
 * \brief   The OdmOrthoPhoto class is used to create an orthographic photo over a given area.
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
    int width, height;
    void parseArguments(int argc, char* argv[]);
    void printHelp();

    void createOrthoPhoto();

    /*!
      * \brief Compute the boundary points so that the entire model fits inside the photo.
      *
      * \param mesh The model which decides the boundary.
      */
    Bounds computeBoundsForModel(const pcl::TextureMesh &mesh);
    
    /*!
      * \brief Creates a transformation which aligns the area for the orthophoto.
      */
    Eigen::Transform<float, 3, Eigen::Affine> getROITransform(float xMin, float yMin) const;

    template <typename T>
    void initBands(int count);

    template <typename T>
    void initAlphaBand();

    template <typename T>
    void finalizeAlphaBand();

    void saveTIFF(const std::string &filename, GDALDataType dataType);
    
    /*!
      * \brief Renders a triangle into the ortho photo.
      *
      *        Pixel center defined as middle of pixel for triangle rasterisation, and in lower left corner for texture look-up.
      *
      * \param texture The texture of the polygon.
      * \param polygon The polygon as athree indices relative meshCloud.
      * \param meshCloud Contains all vertices.
      * \param uvs Contains the texture coordinates for the active material.
      * \param faceIndex The index of the face.
      */
    template <typename T>
    void drawTexturedTriangle(const cv::Mat &texture, const pcl::Vertices &polygon, const pcl::PointCloud<pcl::PointXYZ>::Ptr &meshCloud, const std::vector<Eigen::Vector2f> &uvs, size_t faceIndex);
    
    /*!
      * \brief Sets the color of a pixel in the photo.
      *
      * \param row The row index of the pixel.
      * \param col The column index of the pixel.
      * \param s The u texture-coordinate, multiplied with the number of columns in the texture.
      * \param t The v texture-coordinate, multiplied with the number of rows in the texture.
      * \param texture The texture from which to get the color.
      **/
    template <typename T>
    void renderPixel(int row, int col, float u, float v, const cv::Mat &texture);

    /*!
      * \brief Calculates the barycentric coordinates of a point in a triangle.
      *
      * \param v1 The first triangle vertex.
      * \param v2 The second triangle vertex.
      * \param v3 The third triangle vertex.
      * \param x The x coordinate of the point.
      * \param y The y coordinate of the point.
      * \param l1 The first vertex weight.
      * \param l2 The second vertex weight.
      * \param l3 The third vertex weight.
      */
    void getBarycentricCoordinates(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3, float x, float y, float &l1, float &l2, float &l3) const;

    /*!
      * \brief Check if a given polygon is a sliver polygon.
      *
      * \param v1 The first vertex of the polygon.
      * \param v2 The second vertex of the polygon.
      * \param v3 The third vertex of the polygon.
      */
    bool isSliverPolygon(pcl::PointXYZ v1, pcl::PointXYZ v2, pcl::PointXYZ v3) const;
    
    /*!
      * \brief Check if the model is suitable for ortho photo generation.
      *
      * \param mesh The model.
      * \return True if the model is ok for generating ortho photo.
      */
    bool isModelOk(const pcl::TextureMesh &mesh);

    /*!
      * \brief Loads a model from an .obj file (replacement for the pcl obj loader).
      *
      * \param inputFile Path to the .obj file.
      * \param mesh The model.
      * \return True if model was loaded successfully.
      */
    bool loadObjFile(std::string inputFile, pcl::TextureMesh &mesh, std::vector<pcl::MTLReader> &companions);

    /*!
      * \brief Function is compied straight from the function in the pcl::io module.
      */
    bool readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                     Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                     int &file_version, int &data_type, unsigned int &data_idx,
                     const int offset,
                     std::vector<pcl::MTLReader> &companions);

    Logger          log_;               /**< Logging object. */

    std::vector<std::string> inputFiles;
    std::string     outputFile_;        /**< Path to the destination file. */
    std::string     outputCornerFile_;  /**< Path to the output corner file. */
    std::string     logFile_;           /**< Path to the log file. */
    std::string     bandsOrder;

    float           resolution_;        /**< The number of pixels per meter in the ortho photo. */

    std::vector<void *>    bands;
    std::vector<GDALColorInterp> colorInterps;
    void *alphaBand; // Keep alpha band separate
    int currentBandIndex;

    cv::Mat         depth_;             /**< The depth of the ortho photo as an OpenCV matrix, CV_32F. */
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
