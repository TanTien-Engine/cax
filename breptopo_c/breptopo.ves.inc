#define QUOTE(...) #__VA_ARGS__
static const char* breptopoModuleSource = QUOTE(

foreign class Graph
{
	init() {}

	foreign get_nodes()
	foreign get_edges()

	foreign query_node(x, y)

	foreign layout()
}

foreign class Node
{
	init() {}

	foreign get_pos()
	foreign set_pos(x, y)

	foreign get_face()
}

foreign class TopoAdapter
{
	foreign static build_graph(shape)
}

);