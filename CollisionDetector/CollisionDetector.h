#include "MathUtils.h"

/*---------------------------------------------------------------------------------------------------------------------------------*/

bool point_inside_rectangle(point pnt1, rectangle rec);

point closest_point_from_rectangle(point pnt, rectangle rec);

point segment_intersect_plane(segment seg, rectangle rec, bool &intersect_plane);

point projection_point_to_plane(point pnt, rectangle rec);

point projection_point_to_segment(point pnt, segment seg, bool &on_segment);

void find_point_on_segment_with_right_distance(point pnt, segment seg, double dist, bool &exista_pnt, point &p_out1, point &p_out2);

/*---------------------------------------------------------------------------------------------------------------------------------*/

bool intersect_sphere_sphere(Sphere sph1, Sphere sph2);

bool intersect_parallelipiped_parallelipiped(Parallelepiped prl1, Parallelepiped prl2);

bool intersect_sphere_cilinder(Sphere sph, Cylinder cyl);

bool intersect_sphere_parallelipied(Sphere sph, Parallelepiped prl);

bool intersect_cylinder_cylinder(Cylinder cyl1, Cylinder cyl2); // (Separating Axis Theorem)

/*---------------------------------------------------------------------------------------------------------------------------------*/