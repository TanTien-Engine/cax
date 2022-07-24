#define QUOTE(...) #__VA_ARGS__
static const char* nurbslibModuleSource = QUOTE(

foreign class NurbsLib
{
	foreign static bezier(ctrl_poly, out_poly_num)
}

);