#define QUOTE(...) #__VA_ARGS__
static const char* brepdbModuleSource = QUOTE(



foreign class BrepDB
{
	init() {}
	init(db) {}

	foreign insert(ir)
	foreign query(aabb)
}

);