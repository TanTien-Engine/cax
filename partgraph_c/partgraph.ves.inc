#define QUOTE(...) #__VA_ARGS__
static const char* partgraphModuleSource = QUOTE(

foreign class TopoShape
{
	init() {}
}

foreign class TopoEdge
{
	init() {}
}

foreign class TopoWire
{
	init() {}
}

foreign class TopoFace
{
	init() {}
}

foreign class WireBuilder
{
	init() {}

	foreign add_edge(edge)
	foreign add_wire(wire)

	foreign gen_wire() 
}

foreign class BRepBuilder
{
	foreign static make_edge_from_line(line)
	foreign static make_edge_from_arc(arc)

	foreign static make_wire(edges)
	foreign static make_face(wire)
}

foreign class PrimMaker
{
	foreign static box(length, width, height)
	foreign static cylinder(radius, length)
}

foreign class BRepSelector
{
	foreign static select_face(shape)
}

foreign class TopoAlgo
{
	foreign static fillet(shape, radius)
	foreign static chamfer(shape, dist)

	// sweep
	foreign static extrude(shape, x, y, z)

	// boolean
	foreign static cut(s1, s2)
	foreign static fuse(s1, s2)
	foreign static common(s1, s2)
	foreign static section(s1, s2)

	foreign static translate(shape, x, y, z)
	foreign static mirror(shape, pos, dir)

	// offset
	foreign static draft(shape, dir, angle, len)
	foreign static thick_solid(shape, face, offset)
}

foreign class TopoAdapter
{
	foreign static build_mesh(shape)
	foreign static build_edge_geo(edge)
	foreign static build_wire_geo(wire)

	foreign static shape2wire(shape)
}

);