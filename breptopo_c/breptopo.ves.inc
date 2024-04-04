#define QUOTE(...) #__VA_ARGS__
static const char* breptopoModuleSource = QUOTE(

foreign class TopoGraph
{
	init(shapes) {}

	foreign get_graph()
}

foreign class HistGraph
{
	foreign static get_hist_graph()
	foreign static get_node_uid(shape)
	foreign static query_shapes(uid)
}

);