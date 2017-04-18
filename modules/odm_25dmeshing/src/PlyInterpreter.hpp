#pragma once

#include <utility>
#include <vector>
#include <fstream>
#include <limits>

#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include "CGAL.hpp"

// points, normals
//typedef std::pair<Point3, Vector3> Pwn;

class PlyInterpreter {
	std::vector<Point3>& points;
	long zNormalsDirectionCount;

	public:
	 PlyInterpreter (std::vector<Point3>& points)
	    : points (points), zNormalsDirectionCount(0)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
