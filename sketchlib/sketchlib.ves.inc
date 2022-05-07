#define QUOTE(...) #__VA_ARGS__
static const char* sketchlibModuleSource = QUOTE(

foreign class SketchScene
{
	init() {}

	// geo: geo_id, point_id, shape
	foreign add(cons_id, cons_type, geo1, geo2, value)
	foreign clear()

	foreign solve(ids, shapes)

	foreign get_dof()
}

);