#include "PlyInterpreter.hpp"

// Init and test if input file contains the right properties
bool PlyInterpreter::is_applicable(CGAL::Ply_reader& reader) {
	return reader.does_tag_exist<FT> ("x")
	      && reader.does_tag_exist<FT> ("y")
	      && reader.does_tag_exist<FT> ("z")
	      && reader.does_tag_exist<unsigned char> ("diffuse_red")
	      && reader.does_tag_exist<unsigned char> ("diffuse_green")
	      && reader.does_tag_exist<unsigned char> ("diffuse_blue");
}

// Describes how to process one line (= one point object)
void PlyInterpreter::process_line(CGAL::Ply_reader& reader) {
	FT x = (FT)0., y = (FT)0., z = (FT)0.;
	Color c = {{ 0, 0, 0 }};

	reader.assign (x, "x");
	reader.assign (y, "y");
	reader.assign (z, "z");
	reader.assign (c[0], "diffuse_red");
	reader.assign (c[1], "diffuse_green");
	reader.assign (c[2], "diffuse_blue");
	points.push_back(std::make_pair (Point3 (x, y, z), c));
}
