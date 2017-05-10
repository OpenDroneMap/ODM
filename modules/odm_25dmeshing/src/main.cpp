/*
OpenDroneMap - https://www.opendronemap.org
Copyright (C) 2017 OpenDroneMap Contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Odm25dMeshing.hpp"

/*!
 * \mainpage main OpenDroneMap 2.5D Meshing Module
 *
 *  The OpenDroneMap 2.5D Meshing Module generates a 2.5D mesh using a constrained
 *  delaunay triangulation from any point cloud (points with corresponding normals).
 */

int main(int argc, char** argv)
{
    Odm25dMeshing om;
    return om.run(argc, argv);
}
