#define QUOTE(...) #__VA_ARGS__
static const char* breptopoModuleSource = QUOTE(

foreign class TopoAdapter
{
	foreign static build_graph(shape_list)
}

);