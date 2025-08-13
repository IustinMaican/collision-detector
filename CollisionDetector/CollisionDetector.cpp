
#include <iostream>

#include <stdexcept>

#include <algorithm>

#include "MathUtils.h"

using namespace MathUtils;

bool MathUtils::are_equal(double nr1, double nr2)
{
    if (abs(nr1 - nr2) < EPSILON)
        return true;

    return false;
}

bool MathUtils::comp_points(point pnt1, point pnt2)
{
    if (are_equal(pnt1.linie, pnt2.linie) && are_equal(pnt1.coloana, pnt2.coloana) && are_equal(pnt1.adancime, pnt2.adancime))
        return true;

    return false;
}


double MathUtils::magnitude_vector_squared(Vector v1)
{
    return v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
}


Vector MathUtils::vector_to_unit_vector(Vector v)
{
    double magnitude = sqrt(magnitude_vector_squared(v));
    
    if (are_equal(magnitude, 0))
        throw std::runtime_error("Can not normalize this vector");

    return
    {
        v.x / magnitude,
        v.y / magnitude,
        v.z / magnitude
    };
}


Vector MathUtils::cross_product_vector(Vector v1, Vector v2)
{
    return 
    {
        v1.y*v2.z - v1.z*v2.y,

        v1.z*v2.x - v1.x*v2.z,

        v1.x*v2.y - v1.y*v2.x
    };
}


Vector MathUtils::sum_vector(Vector v1, Vector v2)
{
    return
    {
        v1.x+v2.x,

        v1.y+v2.y,

        v1.z+v2.z
    };
}


Vector MathUtils::diff_vector(Vector v1, Vector v2)
{
    return
    {
        v1.x - v2.x,

        v1.y - v2.y,

        v1.z - v2.z
    };
}


Vector MathUtils::scalar_product_vector(double val, Vector v)
{
    return
    {
        val * v.x,

        val * v.y,

        val * v.z
    };
}


double MathUtils::dot_product_vector(Vector v1, Vector v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}


double MathUtils::pythagoras(double length1, double length2)
{
    return sqrt(length2 * length2 + length1 * length1);
}


double MathUtils::dist_two_points(point pnt1, point pnt2)
{
    double diff_adancime = abs(pnt1.adancime - pnt2.adancime);

    double diff_coloana = abs(pnt1.coloana - pnt2.coloana);

    double cateta1 = pythagoras(diff_adancime, diff_coloana);

    double diff_linie = abs(pnt1.linie - pnt2.linie);

    return pythagoras(cateta1, diff_linie);
}


bool point_inside_rectangle(point pnt1, rectangle rec)
{

    double lin_max = rec.colt_stanga_sus.linie;

    double lin_min = rec.colt_stanga_jos.linie;

    double col_min = rec.colt_stanga_sus.coloana;

    double col_max = rec.colt_dreapta_jos.coloana;

    if (pnt1.linie >= lin_min && pnt1.linie <= lin_max)
    {
        if (pnt1.coloana >= col_min && pnt1.coloana <= col_max)
            return true;
    }

    return false;
}


point closest_point_from_rectangle(point pnt1, rectangle rec)
{

    if (point_inside_rectangle(pnt1, rec))
        return pnt1;

    else
    {
        if (pnt1.linie >= rec.colt_stanga_sus.linie) 
        {
            double mij = (rec.colt_stanga_sus.coloana + rec.colt_dreapta_sus.coloana) / 2;

            if (pnt1.coloana <= mij)
                return rec.colt_stanga_sus;

            else
                return rec.colt_dreapta_sus;
        }

        else if (pnt1.linie <= rec.colt_stanga_jos.linie)
        {
            double mij = (rec.colt_stanga_jos.coloana + rec.colt_dreapta_jos.coloana) / 2;

            if (pnt1.coloana <= mij)
                return rec.colt_stanga_jos;

            else
                return rec.colt_dreapta_jos;
        }

        else
        {
            if (pnt1.coloana <= rec.colt_stanga_sus.coloana)
            {
                double mij = (rec.colt_stanga_sus.linie + rec.colt_stanga_jos.linie);

                if (pnt1.linie <= mij)
                    return rec.colt_stanga_jos;

                else
                    return rec.colt_stanga_sus;
            }

            else
            {
                double mij = (rec.colt_dreapta_sus.linie + rec.colt_dreapta_jos.linie);

                if (pnt1.linie <= mij)
                    return rec.colt_dreapta_jos;

                else
                    return rec.colt_dreapta_sus;
            }
        }
    }
}


point segment_intersect_plane(segment seg, rectangle rec, bool &intersect_plane)
{
    Vector u = 
    {
        rec.colt_dreapta_sus.linie - rec.colt_stanga_sus.linie,

        rec.colt_dreapta_sus.coloana - rec.colt_stanga_sus.coloana,

        rec.colt_dreapta_sus.adancime - rec.colt_stanga_sus.adancime
    };

    Vector v = {
        rec.colt_stanga_jos.linie - rec.colt_stanga_sus.linie,

        rec.colt_stanga_jos.coloana - rec.colt_stanga_sus.coloana,

        rec.colt_stanga_jos.adancime - rec.colt_stanga_sus.adancime
    };

    Vector n = cross_product_vector(u, v);

    Vector d = 
    {
        seg.pnt2.linie - seg.pnt1.linie,

        seg.pnt2.coloana - seg.pnt1.coloana,

        seg.pnt2.adancime - seg.pnt1.adancime
    };

    Vector w = 
    {
        seg.pnt1.linie - rec.colt_stanga_sus.linie,

        seg.pnt1.coloana - rec.colt_stanga_sus.coloana,

        seg.pnt1.adancime - rec.colt_stanga_sus.adancime
    };

    double val1 = dot_product_vector(w, n);
    
    double val2 = dot_product_vector(d, n);

    if(are_equal(val2, 0))
        throw std::runtime_error("Cannot divide a number zero");

    double t = (val1 / val2)*(-1);


    if (t >= 0 && t <= 1)
    {
        intersect_plane = true;

        d = scalar_product_vector(t, d);

        return
        {
            seg.pnt1.linie+d.x,

            seg.pnt1.coloana+d.y,

            seg.pnt1.adancime+d.z
        };
    }

    else
        intersect_plane = false;

    return
    {
        0,
        0,
        0
    };
}


point projection_point_to_plane(point pnt, rectangle rec)
{
    Vector u =
    {
        rec.colt_dreapta_sus.linie - rec.colt_stanga_sus.linie,

        rec.colt_dreapta_sus.coloana - rec.colt_stanga_sus.coloana,

        rec.colt_dreapta_sus.adancime - rec.colt_stanga_sus.adancime
    };

    Vector v =
    {
        rec.colt_stanga_jos.linie - rec.colt_stanga_sus.linie,

        rec.colt_stanga_jos.coloana - rec.colt_stanga_sus.coloana,

        rec.colt_stanga_jos.adancime - rec.colt_stanga_sus.adancime
    };

    Vector n = cross_product_vector(u, v);

    Vector d =
    {
        pnt.linie-rec.colt_stanga_sus.linie,

        pnt.coloana-rec.colt_stanga_sus.coloana,

        pnt.adancime-rec.colt_stanga_sus.adancime
    };

    double val1 = dot_product_vector(d, n);

    if(are_equal(magnitude_vector_squared(n), 0))
        throw std::runtime_error("Cannot divide number by zero");

    val1 /= magnitude_vector_squared(n);

    n = scalar_product_vector(val1, n);

    return
    {
        pnt.linie-n.x,

        pnt.coloana-n.y,

        pnt.adancime-n.z
    };
}


point projection_point_to_segment(point pnt, segment seg, bool &on_segment)
{
    Vector v = 
    {
        seg.pnt2.linie - seg.pnt1.linie,

        seg.pnt2.coloana - seg.pnt1.coloana,

        seg.pnt2.adancime - seg.pnt1.adancime
    };

    Vector w = 
    {
        pnt.linie - seg.pnt1.linie,

        pnt.coloana - seg.pnt1.coloana,

        pnt.adancime - seg.pnt1.adancime
    };

    double val1 = dot_product_vector(w, v);

    double val2 = dot_product_vector(v, v);

    if(are_equal(val2, 0))
        throw std::runtime_error("Cannot divide number by zero");

    double t = val1 / val2;

    if (are_equal(t, 0)) 
    {
        on_segment = true;

        return seg.pnt1;
    }

    else if (are_equal(t, 1))
    {
        on_segment = true;

        return seg.pnt2;
    }

    else
    {
        if (t >= 0 && t <= 1)
            on_segment = true;

        else
            on_segment = false;

        v = scalar_product_vector(t, v);

        point ras = 
        {
            seg.pnt1.linie + v.x,

            seg.pnt1.coloana + v.y,

            seg.pnt1.adancime + v.z
        };

        return ras;
    }
}


solutions_of_equation MathUtils::solve_equation_grade2(double a, double b, double c, bool &exista_sol)
{
    solutions_of_equation ras;
    if (are_equal(a, 0))
        throw std::runtime_error("Cannot divide number by zero");

    double delta = b * b - 4 * a * c;

    if (delta < 0) 
        exista_sol = false;

    else
    {
        delta = sqrt(delta);

        double sol1 = (-b + delta) / (2 * a);

        double sol2 = (-b - delta) / (2 * a);

        ras.sol1 = sol1;

        ras.sol2 = sol2;

        if (sol1 >= 0 && sol1 <= 1) 
        {
            ras.sol1 = sol1;

            exista_sol = true;

            if (sol2 >= 0 && sol2 <= 1)
                ras.sol2 = sol2;

            else
                ras.sol2 = sol1;
        }

        else 
        {
            if (sol2 >= 0 && sol2 <= 1) 
            {
                ras.sol2 = sol2;

                ras.sol1 = sol2;

                exista_sol = true;
            }

            else
                exista_sol = false;
        }
    }
    return ras;
}


void find_point_on_segment_with_right_distance(point pnt, segment seg, double dist, bool &exista_pnt,
    point &p_out1, point &p_out2)
{
    Vector d = 
    {
        seg.pnt2.linie - seg.pnt1.linie,

        seg.pnt2.coloana - seg.pnt1.coloana,

        seg.pnt2.adancime - seg.pnt1.adancime
    };

    Vector copie_d = 
    {
        seg.pnt2.linie - seg.pnt1.linie,

        seg.pnt2.coloana - seg.pnt1.coloana,

        seg.pnt2.adancime - seg.pnt1.adancime
    };

    Vector v = 
    {
        seg.pnt1.linie - pnt.linie,

        seg.pnt1.coloana - pnt.coloana,

        seg.pnt1.adancime - pnt.adancime
    };

    double a = dot_product_vector(d, d);

    double b = dot_product_vector(v, d)*2;

    double c = dot_product_vector(v, v) - dist * dist;

    solutions_of_equation ras;

    bool exista_sol = false;

    ras = solve_equation_grade2(a, b, c, exista_sol);

    exista_pnt = exista_sol;

    d = scalar_product_vector(ras.sol1, d);

    p_out1 = 
    {
        pnt.linie + d.x,

        pnt.coloana + d.y,

        pnt.adancime + d.z 
    };

    copie_d = scalar_product_vector(ras.sol2, copie_d);

    p_out2 = 
    {
        pnt.linie + copie_d.x,

        pnt.coloana + copie_d.y,

        pnt.adancime + copie_d.z
    };
}


bool intersect_sphere_sphere(Sphere sph1, Sphere sph2)
{
    double dist = dist_two_points(sph1.centru_sfera, sph2.centru_sfera);

    if (dist <= sph1.raza_sfera + sph2.raza_sfera)
        return true;

    return false;
}


bool intersect_parallelipiped_parallelipiped(Parallelepiped prl1, Parallelepiped prl2)
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 12; j++) 
        {
            bool intersect_plane = false;

            point ras;
            ras = segment_intersect_plane(prl1.seg[j], prl2.faces_paral[i], intersect_plane);

            if (intersect_plane)
            {
                point closest_to_rectangle = closest_point_from_rectangle(ras, prl2.faces_paral[i]);

                if (comp_points(closest_to_rectangle, ras))
                    return true;
            }
        }
    }

    return false;
}


bool intersect_sphere_parallelipied(Sphere sph, Parallelepiped prl)
{

    for (int i = 0; i < 6; i++)
    {
        point ras;

        ras = projection_point_to_plane(sph.centru_sfera, prl.faces_paral[i]);

        point ras_fin;

        ras_fin = closest_point_from_rectangle(ras, prl.faces_paral[i]);

        double dist = dist_two_points(ras_fin, sph.centru_sfera);

        if (dist <= sph.raza_sfera)
            return true;
    }

    return false;
}


bool intersect_sphere_cilinder(Sphere sph, Cylinder cyl)
{
    bool on_segment = false;

    point ras;

    ras = projection_point_to_segment(sph.centru_sfera, cyl.axa_Cylinder, on_segment);

    if (on_segment == true)
    {
        double dist = dist_two_points(sph.centru_sfera, ras);

        if (dist <= sph.raza_sfera + cyl.raza_Cylinder)
            return true;
    }
    else
    {
        bool intersect_circle = false;

        point ras1;

        rectangle plane = 
        {
            cyl.low_face.nord_point,

            cyl.low_face.east_point,

            cyl.low_face.south_point,

            cyl.low_face.west_point
        };

        point highest_sphere_point =
        {
            sph.centru_sfera.linie + sph.raza_sfera,

            sph.centru_sfera.coloana,

            sph.centru_sfera.adancime
        };

        point lowest_sphere_point = 
        {
            sph.centru_sfera.linie - sph.raza_sfera,

            sph.centru_sfera.coloana,

            sph.centru_sfera.adancime
        };

        segment sphere_axis = 
        {
            highest_sphere_point,

            lowest_sphere_point
        };
        
        ras1 = segment_intersect_plane(sphere_axis, plane, intersect_circle);

        if (intersect_circle)
        {
            segment seg = 
            {
                ras1,
                cyl.low_face.center
            };

            bool exista_pnt = false;

            point p1_out, p2_out;

            find_point_on_segment_with_right_distance(sph.centru_sfera, seg, sph.raza_sfera, exista_pnt, p1_out, p2_out);

            if (dist_two_points(ras1, p1_out) + cyl.raza_Cylinder >= dist_two_points(ras1, cyl.low_face.center))
                return true;

            if (dist_two_points(ras1, p2_out) + cyl.raza_Cylinder >= dist_two_points(ras1, cyl.low_face.center))
                return true;
        }
        else
        {
            intersect_circle = false;

            rectangle plane = 
            {
                cyl.high_face.nord_point,

                cyl.high_face.east_point,

                cyl.high_face.south_point,

                cyl.high_face.west_point
            };

            ras1 = segment_intersect_plane(sphere_axis, plane, intersect_circle); 

            if (intersect_circle)
            {
                segment seg = 
                {
                    ras1,

                    cyl.high_face.center
                };

                bool exista_pnt = false;

                point p1_out, p2_out;

                find_point_on_segment_with_right_distance(sph.centru_sfera, seg, sph.raza_sfera, exista_pnt, p1_out, p2_out);

                if (dist_two_points(ras1, p1_out) + cyl.raza_Cylinder >= dist_two_points(ras1, cyl.low_face.center))
                    return true;

                if (dist_two_points(ras1, p2_out) + cyl.raza_Cylinder >= dist_two_points(ras1, cyl.low_face.center))
                    return true;
            }
        }
    }

    return false;
}


bool intersect_cylinder_cylinder(Cylinder cyl1, Cylinder cyl2)
{
    Vector a[3];

    int limit = 2;

    a[0] =
    {
        cyl1.axa_Cylinder.pnt1.linie - cyl1.axa_Cylinder.pnt2.linie,

        cyl1.axa_Cylinder.pnt1.coloana - cyl1.axa_Cylinder.pnt2.coloana,

        cyl1.axa_Cylinder.pnt1.adancime - cyl1.axa_Cylinder.pnt2.adancime
    };

    a[1] =
    {
        cyl2.axa_Cylinder.pnt1.linie - cyl2.axa_Cylinder.pnt2.linie,

        cyl2.axa_Cylinder.pnt1.coloana - cyl2.axa_Cylinder.pnt2.coloana,

        cyl2.axa_Cylinder.pnt1.adancime - cyl2.axa_Cylinder.pnt2.adancime
    };

    if (!are_equal(0, magnitude_vector_squared(cross_product_vector(a[0], a[1]))))
    {
        limit++;
        a[2] = cross_product_vector(a[0], a[1]);
    }

    for (int i = 0; i < limit; i++)
    {
        Vector axis = vector_to_unit_vector(a[i]);

        Vector p1=
        {
            cyl1.axa_Cylinder.pnt1.linie,

            cyl1.axa_Cylinder.pnt1.coloana,
            
            cyl1.axa_Cylinder.pnt1.adancime
        };

        Vector p2 =
        {
            cyl1.axa_Cylinder.pnt2.linie,

            cyl1.axa_Cylinder.pnt2.coloana,

            cyl1.axa_Cylinder.pnt2.adancime
        };

        Vector unit_axis = vector_to_unit_vector(a[0]);

        double p1_val = dot_product_vector(p1, axis);

        double p2_val = dot_product_vector(p2, axis);

        double product = dot_product_vector(unit_axis, axis);

        if (product < -1.0)
            product = -1.0;

        else if (product > 1.0)
            product = 1.0;

        if (product*product > 1)
            throw std::domain_error("Can not take square root of a negative number, look in function intersect_cylinder_cylinder, first sqrt");

        double r = cyl1.raza_Cylinder * sqrt(1 - product * product);

        double min_cyl1 = std::min(p1_val, p2_val) - r;

        double max_cyl1 = std::max(p1_val, p2_val) + r;

        p1 =
        {
            cyl2.axa_Cylinder.pnt1.linie,

            cyl2.axa_Cylinder.pnt1.coloana,

            cyl2.axa_Cylinder.pnt1.adancime
        };

        p2 =
        {
            cyl2.axa_Cylinder.pnt2.linie,

            cyl2.axa_Cylinder.pnt2.coloana,

            cyl2.axa_Cylinder.pnt2.adancime
        };

        unit_axis = vector_to_unit_vector(a[1]);

        p1_val = dot_product_vector(p1, axis);

        p2_val = dot_product_vector(p2, axis);

        product = dot_product_vector(unit_axis, axis);

        if (product < -1.0)
            product = -1.0;

        else if (product > 1.0)
            product = 1.0;

        if (product*product > 1)
            throw std::domain_error("Can not take square root of a negative number, look in function intersect_cylinder_cylinder, second sqrt");

        r = cyl2.raza_Cylinder * sqrt(1 - product * product);

        double min_cyl2 = std::min(p1_val, p2_val) - r;

        double max_cyl2 = std::max(p1_val, p2_val) + r;

        if (min_cyl1 <= max_cyl2 && min_cyl2 <= max_cyl1)
            continue;

        else if (min_cyl2 <= max_cyl1 && min_cyl1 <= max_cyl2)
            continue;

        else
            return false;
    }

    return true;
}


int main()
{
    try {
        Cylinder cyl1, cyl2;

        cyl1.init();

        cyl2.init();

        std::cout << intersect_cylinder_cylinder(cyl1, cyl2);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what();
    }
    return 0;
}

