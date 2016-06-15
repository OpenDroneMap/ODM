#pragma once

// STL
#include <string>
#include <iostream>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

// Logging
#include "Logger.hpp"

/*!
 * \brief   The OdmMeshing class is used to create a triangulated mesh using the Poisson method.
 *          The class reads an oriented point cloud (coordinates and normals) from a PLY ascii
 *          file and outputs the resulting welded manifold mesh on the form of an ASCII PLY-file.
 *          The class uses file read and write functions from pcl.
 */
class OdmMeshing
{
public:
    OdmMeshing();
    ~OdmMeshing();

    /*!
     * \brief   run     Runs the meshing functionality using the provided input arguments.
     *                  For a list of accepted arguments, please see the main page documentation or
     *                  call the program with parameter "-help".
     * \param   argc    Application argument count.
     * \param   argv    Argument values.
     * \return  0       If successful.
     */
    int run(int argc, char **argv);

private:

    /*!
     * \brief parseArguments    Parses command line arguments.
     * \param   argc    Application argument count.
     * \param   argv    Argument values.
     */
    void parseArguments(int argc, char** argv);

    /*!
     * \brief createMesh    Sets up the pcl::Poisson meshing class using provided arguments and calls
     *                      it to start the meshing.
     */
    void createMesh();

    /*!
     * \brief loadPoints    Loads a PLY ascii file with points and normals from file.
     */
    void loadPoints();

    /*!
     * \brief decimateMesh  Performs post-processing on the form of quadric decimation to generate a mesh
     *                      that has a higher density in areas with a lot of structure.
     */
    void decimateMesh();

    /*!
     * \brief writePlyFile  Writes the mesh to file on the Ply format.
     */
    void writePlyFile();

    /*!
     * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
     */
    void printHelp();

    /*!
     * \brief calcTreeDepth Attepts to calculate the depth of the tree using the point cloud.
     *                      The function makes the assumption points are located roughly in a plane
     *                      (fairly reasonable for ortho-terrain photos) and tries to generate a mesh using
     *                      an octree with an appropriate resolution.
     * \param nPoints       The total number of points in the input point cloud.
     * \return              The calcualated octree depth.
     */
    int calcTreeDepth(size_t nPoints);

    Logger log_;                /**< Logging object. */

    pcl::Poisson<pcl::PointNormal>::Ptr meshCreator_;    /**< PCL poisson meshing class. */

    pcl::PointCloud<pcl::PointNormal>::Ptr points_; /**< Input point and normals. */
    pcl::PolygonMeshPtr mesh_;                      /**< PCL polygon mesh. */
    pcl::PolygonMeshPtr decimatedMesh_;             /**< Decimated polygon mesh. */

    std::string inputFile_;     /**< Path to a file containing points and normals. */
    std::string outputFile_;    /**< Path to the destination file. */
    std::string logFilePath_;       /**< Path to the log file. */

    unsigned int maxVertexCount_;  /**< Desired output vertex count. */
    unsigned int treeDepth_;    /**< Depth of octree used for reconstruction. */

    double samplesPerNode_;     /**< Samples per octree node.*/
    double solverDivide_;       /**< Depth at which the Laplacian equation solver is run during surface estimation.*/
    double decimationFactor_;   /**< Percentage of points to remove when decimating the mesh. */
};

/*!
 * \brief The OdmMeshingException class
 */
class OdmMeshingException : public std::exception
{

public:
    OdmMeshingException() : message("Error in OdmMeshing") {}
    OdmMeshingException(std::string msgInit) : message("Error in OdmMeshing:\n" + msgInit) {}
    ~OdmMeshingException() throw() {}
    virtual const char* what() const throw() {return message.c_str(); }

private:
    std::string message;    /**< The error message **/
};
