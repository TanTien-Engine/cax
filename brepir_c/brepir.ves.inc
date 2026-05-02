#define QUOTE(...) #__VA_ARGS__
static const char* brepirModuleSource = QUOTE(

foreign class BrepIR
{
	foreign static save(shape, filepath)
	foreign static load(filepath)
}

);