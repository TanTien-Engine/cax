#define QUOTE(...) #__VA_ARGS__
static const char* partgraphModuleSource = QUOTE(

foreign class TopoShape
{
	init() {}
}

foreign class BRepPrimAPI
{
	foreign static box(length, width, height)
}

foreign class BRepFilletAPI
{
	foreign static make_fillet(topo, thickness)
}

foreign class MeshBuilder
{
	foreign static build_from_topo(topo)
}

);