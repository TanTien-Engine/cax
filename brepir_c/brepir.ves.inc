#define QUOTE(...) #__VA_ARGS__
static const char* brepirModuleSource = QUOTE(

foreign class BrepIR
{
	init() {}

	foreign save(filepath)
	foreign load(filepath)

	foreign serialize(shape)
	foreign deserialize()
}

);