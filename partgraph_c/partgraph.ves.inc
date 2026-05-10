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
	foreign static make_shell(faces)

	foreign static make_compound(shapes)
}

foreign class PrimMaker
{
	foreign static plane(x, y, z, nx, ny, nz, op_id)

	foreign static box(length, width, height, op_id)
	foreign static cylinder(radius, length, op_id)
	foreign static cone(radius1, radius2, height, op_id)
	foreign static sphere(radius, op_id)
	foreign static sphere_with_angle(radius, angle, op_id)
	foreign static torus(r1, r2, op_id)
	foreign static torus_with_angle(r1, r2, angle, op_id)

	foreign static threading(thickness, height, op_id)
}

foreign class BRepSelector
{
	foreign static select_face(shape, pos, dir)
	foreign static select_edge(shape, pos, dir)
}

foreign class BRepTools
{
	foreign static find_edge_idx(shape, key)
	foreign static find_edge_key(shape, idx)

	foreign static find_face_idx(shape, key)
	foreign static find_face_key(shape, idx)

	foreign static map_shells(shape)
	foreign static map_faces(shape)
	foreign static map_edges(shape)
}

foreign class TopoAlgo
{
	foreign static fillet(shape, radius, edges, op_id)
	foreign static chamfer(shape, dist, edges, op_id)

	// sweep
	foreign static extrude(shape, x, y, z, op_id)

	// boolean
	foreign static split(base, tool, op_id)
	foreign static cut(s1, s2, op_id)
	foreign static fuse(s1, s2, op_id)
	foreign static common(s1, s2, op_id)
	foreign static section(s1, s2, op_id)

	foreign static sew(s1, s2, op_id)
	foreign static unify_same_domain(shape, op_id)

	foreign static translate(shape, x, y, z, op_id)
	foreign static rotate(shape, pos, dir, angle, op_id)
	foreign static scale(shape, center, factor, op_id)
	foreign static transform(shape, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, op_id)
	foreign static mirror(shape, pos, dir, op_id)

	// offset
	foreign static draft(shape, dir, angle, len, op_id)
	foreign static thick_solid(shape, face, offset, op_id)
	foreign static thru_sections(wires, op_id)
	foreign static offset_shape(shape, offset, is_solid, op_id)

	foreign static extrude_ex(shape, dx, dy, dz, dist1, dist2, end1, end2, ref, op_id)
	foreign static revolve(shape, ox, oy, oz, dx, dy, dz, angle, is_full, op_id)
	foreign static sweep(profile, path, is_solid, op_id)
	foreign static linear_pattern(base, d1x, d1y, d1z, count1, sp1, d2x, d2y, d2z, count2, sp2, op_id)
	foreign static circular_pattern(base, ox, oy, oz, dx, dy, dz, count, angle, op_id)
	foreign static hole_wizard(body, px, py, pz, dx, dy, dz, diameter, depth, hole_type, cb_dia, cb_depth, cs_dia, cs_angle, op_id)
	foreign static variable_fillet(shape, edges, params, op_id)
	foreign static sweep_with_guide(profile, path, guides, is_solid, op_id)
	foreign static rib(body, profile, dx, dy, dz, thickness, is_symmetric, op_id)
}

foreign class TopoAdapter
{
	foreign static build_mesh(dev, shape)

	foreign static build_edge_geo(edge)
	foreign static build_wire_geo(wire)

	foreign static shape2wire(shape)
}

foreign class GlobalConfig
{
	foreign static get_topo_naming()
	foreign static get_version_tree()
}

);