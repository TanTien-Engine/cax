#define QUOTE(...) #__VA_ARGS__
static const char* breptopoModuleSource = QUOTE(

foreign class TopoGraph
{
	init(shapes) {}

	foreign get_graph()
}

foreign class HistGraph
{
	init() {}

	foreign get_hist_graph()
	foreign get_next_op_id()
	foreign get_node_uid(shape)
	foreign query_shapes(uid)
}

foreign class CompGraph
{
	init() {}

	foreign get_graph()

	foreign eval(node_idx, hist)
	foreign update(node_idx, val)

	foreign add_integer_node(int, desc)
	foreign add_number_node(num, desc)
	foreign add_number3_node(x, y, z, desc)
	foreign add_boolean_node(bool, desc)
	foreign add_shape_node(shp, desc)

	foreign add_box_node(length, width, height)
	foreign add_translate_node(shape, offset)
	foreign add_offset_node(shape, offset, is_solid)
	foreign add_cut_node(shp1, shp2)

	foreign add_selector_node(shape, uid)
	foreign add_merge_node(array)
}

);