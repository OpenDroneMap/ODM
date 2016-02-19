// Insert license here.

// Include meshing source code.
#include "OdmMeshing.hpp"

/*!
 * \mainpage main OpenDroneMap Meshing Module
 *
 *  The OpenDroneMap Meshing Module generates a welded, manifold mesh using the Poisson
 *  surface reconstruction algorithm from any oriented point cloud (points with corresponding normals).
 *
 */

int main(int argc, char** argv)
{

    OdmMeshing meshCreator;
    return meshCreator.run(argc, argv);

}
