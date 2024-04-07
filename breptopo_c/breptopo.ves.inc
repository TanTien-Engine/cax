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
	foreign static get_next_op_id()
	foreign static get_node_uid(shape)
	foreign static query_shapes(uid)
}

foreign class CompGraph
{
	init() {}

	foreign get_graph()

	foreign eval(node_idx)

	foreign add_number_node(num)
	foreign add_boolean_node(bool)
	foreign add_shape_node(shp)

	foreign add_box_node(length, width, height)
	foreign add_offset_node(shape, offset, is_solid)
	foreign add_merge_node(array)
}

);