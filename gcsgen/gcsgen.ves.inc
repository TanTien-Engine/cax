#define QUOTE(...) #__VA_ARGS__
static const char* gcsgenModuleSource = QUOTE(

foreign class GCS_Scene
{
	init() {}

	foreign add(cons_id, cons_type, geo1_id, goe1_shape, geo2_id, geo2_shape, value)
	foreign clear()

	foreign solve(ids, shapes)
}

);