#include "PlyInterpreter.hpp"

// Init and test if input file contains the right properties
bool PlyInterpreter::is_applicable(CGAL::Ply_reader& reader) {
	return reader.does_tag_exist<FT> ("x")
	      && reader.does_tag_exist<FT> ("y")
	      && reader.does_tag_exist<FT> ("z")
		  && reader.does_tag_exist<FT> ("nx")
		  && reader.does_tag_exist<FT> ("ny")
		  && reader.does_tag_exist<FT> ("nz");
}

// Describes how to process one line (= one point object)
void PlyInterpreter::process_line(CGAL::Ply_reader& reader) {
	FT x = (FT)0., y = (FT)0., z = (FT)0.,
		nx = (FT)0., ny = (FT)0., nz = (FT)0.;

	reader.assign (x, "x");
	reader.assign (y, "y");
	reader.assign (z, "z");
	reader.assign (nx, "nx");
	reader.assign (ny, "ny");
	reader.assign (nz, "nz");

	Point3 p(x, y, z);
//	Vector3 n(nx, ny, nz);

	if (nz >= 0 && zNormalsDirectionCount < std::numeric_limits<long>::max()){
		zNormalsDirectionCount++;
	}else if (nz < 0 && zNormalsDirectionCount > std::numeric_limits<long>::min()){
		zNormalsDirectionCount--;
	}

//	points.push_back(std::make_pair(p, n));
	points.push_back(p);
}

bool PlyInterpreter::flip_faces(){
	return zNormalsDirectionCount < 0;
}
