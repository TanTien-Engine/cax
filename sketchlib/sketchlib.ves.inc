#define QUOTE(...) #__VA_ARGS__
static const char* sketchlibModuleSource = QUOTE(

foreign class SketchScene
{
	init() {}

	// geo: geo_id, point_id, shape
	foreign add_cons_2(cons_id, cons_type, geo1, geo2, value, driving)
	foreign add_cons_4(cons_id, cons_type, geo1, geo2, geo3, geo4, value, driving)

	foreign clear()

	foreign solve(ids, shapes)

	foreign get_dof()
}

);