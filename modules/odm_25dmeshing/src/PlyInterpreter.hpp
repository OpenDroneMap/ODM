#pragma once

#include <utility>
#include <vector>
#include <fstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;

// points, normals
typedef std::pair<Point3, Vector3> Pwn;

class PlyInterpreter {
	std::vector<Pwn>& points;
	int zNormalsDirectionCount;

	public:
	 PlyInterpreter (std::vector<Pwn>& points)
	    : points (points), zNormalsDirectionCount(0)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
