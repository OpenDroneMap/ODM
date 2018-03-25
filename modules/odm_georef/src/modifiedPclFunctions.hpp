#pragma once

// STL
#include <iostream>
#include <fstream>

// PCL
#include <pcl/point_types.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/io/obj_io.h>

int saveOBJFile(const std::string &file_name, const pcl::TextureMesh &tex_mesh, unsigned precision);

bool getPixelCoordinates(const pcl::PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, pcl::PointXY &UV_coordinates);

bool isFaceProjected (const pcl::TextureMapping<pcl::PointXYZ>::Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3);

void getTriangleCircumscribedCircleCentroid(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);

bool checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt);
