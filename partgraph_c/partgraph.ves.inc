#define QUOTE(...) #__VA_ARGS__
static const char* partgraphModuleSource = QUOTE(

foreign class TopoShape
{
	init() {}
}

foreign class PrimMaker
{
	foreign static box(length, width, height)
	foreign static cylinder(radius, length)
}

foreign class TopoAlgo
{
	foreign static make_fillet(topo, thickness)
	foreign static make_chamfer(topo, dist)

	foreign static cut(s1, s2)
	foreign static fuse(s1, s2)
	foreign static section(s1, s2)

	foreign static translate(shape, x, y, z)
}

foreign class MeshBuilder
{
	foreign static build_from_topo(topo)
}

);