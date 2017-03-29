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

// Point with normal vector stored as a std::pair.
typedef std::pair<Point3, Vector3> Pwn;
// Color is red/green/blue array
typedef CGAL::cpp11::array<unsigned char, 3> Color;

class PlyInterpreter {
	 std::vector<Pwn>& points;
	 std::vector<Color>& colors;

	public:
	 PlyInterpreter (std::vector<Pwn>& points,
			    std::vector<Color>& colors)
	    : points (points), colors (colors)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
};
