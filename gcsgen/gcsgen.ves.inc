#define QUOTE(...) #__VA_ARGS__
static const char* gcsgenModuleSource = QUOTE(

foreign class GCS_Constraint
{
	init(type, geo0, geo1, value) {}

	foreign set_value(v)
}

foreign class GCS_Scene
{
	init() {}

	foreign add(cons)
	foreign clear()

	foreign solve()
}

);