#define QUOTE(...) #__VA_ARGS__
static const char* nurbslibModuleSource = QUOTE(

foreign class NurbsLib
{
	foreign static bezier(ctrl_poly, out_poly_num)
	foreign static bspline(ctrl_poly, order, out_poly_num)
	foreign static rbspline(ctrl_poly, order, out_poly_num)
	foreign static bezsurf(ctl_pts, npts, mpts, p1, p2)
	foreign static bspsurf(ctl_pts, order_u, order_v, npts, mpts, p1, p2)
	foreign static rbspsurf(ctl_pts, order_u, order_v, npts, mpts, p1, p2)
}

);