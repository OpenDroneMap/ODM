#include <vector>
#include <algorithm>

#include<CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/IO/Polyhedron_iostream.h>

#include "CGAL.hpp"

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class PolyhedronBuilder : public CGAL::Modifier_base<HDS> {
public:
 std::vector<float> &vertices;
 std::vector<int>    &vertexIndices;

 PolyhedronBuilder( std::vector<float> &vertices, std::vector<int> &vertexIndices )
 	 : vertices(vertices), vertexIndices(vertexIndices) {}

	void operator()( HDS& hds) {
		typedef typename HDS::Vertex Vertex;
		typedef typename Vertex::Point Point;

		CGAL::Polyhedron_incremental_builder_3<HDS> builder( hds, true);
		builder.begin_surface( vertices.size() / 3, vertexIndices.size() / 3 );

		for(size_t i = 0; i < vertices.size(); i+=3 ){
			builder.add_vertex(Point(vertices[i+0], vertices[i+1], vertices[i+2]));
		}

		for(size_t i = 0; i < vertexIndices.size(); i+=3){
			builder.begin_facet();
			builder.add_vertex_to_facet(vertexIndices[i+0]);
			builder.add_vertex_to_facet(vertexIndices[i+1]);
			builder.add_vertex_to_facet(vertexIndices[i+2]);
			builder.end_facet();
		}

		// finish up the surface
		builder.end_surface();
	}
};


