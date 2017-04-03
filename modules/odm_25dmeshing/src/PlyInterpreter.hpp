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

// Point with normal vector stored as a std::pair.
// Color is red/green/blue array
typedef CGAL::cpp11::array<unsigned char, 3> Color;
typedef std::pair<Point3, Color> Pwc;

class PlyInterpreter {
	 std::vector<Pwc>& points;

	public:
	 PlyInterpreter (std::vector<Pwc>& points)
	    : points (points)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
};
