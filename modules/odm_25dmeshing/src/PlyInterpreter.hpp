#pragma once

#include <utility>
#include <vector>
#include <fstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <boost/tuple/tuple.hpp>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;


// Point with normal vector stored as a std::pair.
// Color is red/green/blue array
typedef CGAL::cpp11::array<unsigned char, 3> Color;

// points, normals and colors
typedef boost::tuple<Point3, Vector3, Color> PointNormalColor;

class PlyInterpreter {
	 std::vector<PointNormalColor>& points;

	public:
	 PlyInterpreter (std::vector<PointNormalColor>& points)
	    : points (points)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
};
