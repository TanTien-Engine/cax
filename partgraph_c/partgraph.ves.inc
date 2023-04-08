#define QUOTE(...) #__VA_ARGS__
static const char* partgraphModuleSource = QUOTE(
	
foreign class CylindricalSurface
{
	init(pos, dir, radius) {}
}

foreign class EllipseCurve
{
	init(pos, dir, major, minor) {}
}

foreign class TrimmedCurve
{
	init(curve, u1, u2) {}
}

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
	foreign static make_edge_from_curve_surf(c, s)

	foreign static make_wire(edges)
	foreign static make_face(wire)

	foreign static make_compound(shapes)
}

foreign class PrimMaker
{
	foreign static box(length, width, height)
	foreign static cylinder(radius, length)
	foreign static cone(radius1, radius2, height)
	foreign static sphere(radius)
	foreign static sphere_with_angle(radius, angle)
	foreign static torus(r1, r2)
	foreign static torus_with_angle(r1, r2, angle)

	foreign static threading(thickness, height)
}

foreign class BRepSelector
{
	foreign static select_face(shape, pos, dir)
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
	foreign static thru_sections(wires)

}

foreign class TopoAdapter
{
	foreign static build_mesh(shape)
	foreign static build_edge_geo(edge)
	foreign static build_wire_geo(wire)

	foreign static shape2wire(shape)
}

);