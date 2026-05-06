#define QUOTE(...) #__VA_ARGS__
static const char* brepdbModuleSource = QUOTE(

foreign class BrepIR
{
	init() {}

	foreign save(filepath)
	foreign load(filepath)

	foreign serialize(shape)
	foreign deserialize()
}

foreign class BrepDB
{
	init() {}
	init(db) {}

	foreign insert(ir)
	foreign query(aabb)
}

);