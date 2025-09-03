
#include <iostream>

#include <stdexcept>

#include <algorithm>

#include "MathUtils.h"

using namespace MathUtils;

bool MathUtils::are_equal(double nr1, double nr2)
{
    if (fabs(nr1 - nr2) < EPSILON)
        return true;

    return false;
}


double MathUtils::magnitude_vector_squared(const Vector& v1)
{
    return v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
}


Vector MathUtils::vector_to_unit_vector(const Vector& v)
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


Vector MathUtils::cross_product_vector(const Vector& v1, const Vector& v2)
{
    return 
    {
        v1.y*v2.z - v1.z*v2.y,

        v1.z*v2.x - v1.x*v2.z,

        v1.x*v2.y - v1.y*v2.x
    };
}




Vector MathUtils::diff_vector(const Vector& v1, const Vector& v2)
{
    return
    {
        v1.x - v2.x,

        v1.y - v2.y,

        v1.z - v2.z
    };
}


Vector MathUtils::scalar_product_vector(double val, const Vector& v)
{
    return
    {
        val * v.x,

        val * v.y,

        val * v.z
    };
}


double MathUtils::dot_product_vector(const Vector& v1, const Vector& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}


double MathUtils::pythagoras(double length1, double length2)
{
    return sqrt(length2 * length2 + length1 * length1);
}


double MathUtils::dist_two_points(const point& pnt1, const point& pnt2)
{
    double diff_depth = abs(pnt1.depth - pnt2.depth);

    double diff_column = abs(pnt1.column - pnt2.column);

    double cateta1 = pythagoras(diff_depth, diff_column);

    double diff_line = abs(pnt1.line - pnt2.line);

    return pythagoras(cateta1, diff_line);
}


bool intersect_sphere_sphere(Sphere sph1, Sphere sph2)
{
    double dist = dist_two_points(sph1.center_sphere, sph2.center_sphere);

    if (dist <= sph1.radius_sphere + sph2.radius_sphere)
        return true;

    return false;
}


bool intersect_parallelepiped_parallelepiped(Parallelepiped prl1, Parallelepiped prl2)
{
    Vector v[15], v1[3], v2[3];

    v1[0] = prl1.u, v1[1] = prl1.v, v1[2] = prl1.w;
    v2[0] = prl2.u, v2[1] = prl2.v, v2[2] = prl2.w;

    double length;
    int cnt = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Vector aux= cross_product_vector(v1[i], v2[j]);

            length = sqrt(magnitude_vector_squared(aux));

            if (!are_equal(length, 0))
                v[cnt++] = aux;
        }
    }
    for (int i = 0; i < 3; i++)
    {
        length = sqrt(magnitude_vector_squared(v1[i]));

        if (!are_equal(length, 0))
            v[cnt++] = v1[i];

        length = sqrt(magnitude_vector_squared(v2[i]));

        if (!are_equal(length, 0))
            v[cnt++] = v2[i];
    }

    for (int i = 0; i < cnt; i++)
        v[i] = vector_to_unit_vector(v[i]);

    Vector C1 =
    {
        prl1.P.line,
        prl1.P.column,
        prl1.P.depth
    };

    Vector C2 =
    {
        prl2.P.line,
        prl2.P.column,
        prl2.P.depth
    };

    for (int i = 0; i < cnt; i++)
    {
        double c1 = dot_product_vector(C1, v[i]);

        double r1 = prl1.hx*fabs(dot_product_vector(prl1.u, v[i])) +
            prl1.hy*fabs(dot_product_vector(prl1.v, v[i])) +
            prl1.hz*fabs(dot_product_vector(prl1.w, v[i]));

        double c2 = dot_product_vector(C2, v[i]);

        double r2 = prl2.hx*fabs(dot_product_vector(prl2.u, v[i])) +
            prl2.hy*fabs(dot_product_vector(prl2.v, v[i])) +
            prl2.hz*fabs(dot_product_vector(prl2.w, v[i]));

        if (fabs(c1 - c2) > r1 + r2)
            return false;
    }
    return true;
}


bool intersect_sphere_parallelipied(Sphere sph, Parallelepiped prl)
{
    Vector d =
    {
        sph.center_sphere.line - prl.P.line,
        sph.center_sphere.column - prl.P.column,
        sph.center_sphere.depth - prl.P.depth
    };
    
    double cx = dot_product_vector(d, prl.u);

    double cy = dot_product_vector(d, prl.v);
    
    double cz = dot_product_vector(d, prl.w);

    double qx, qy, qz;

    if (cx < -prl.hx)
        qx = -prl.hx;
    else if (cx > prl.hx)
        qx = prl.hx;
    else
        qx = cx;

    if (cy < -prl.hy)
        qy = -prl.hy;
    else if (cy > prl.hy)
        qy = prl.hy;
    else
        qy = cy;

    if (cz < -prl.hz)
        qz = -prl.hz;
    else if (cz > prl.hz)
        qz = prl.hz;
    else
        qz = cz;

    double dx, dy, dz, dist;

    dx = cx - qx;
    dy = cy - qy;
    dz = cz - qz;
    dist = dx * dx + dy * dy + dz * dz;

    if (dist <= sph.radius_sphere*sph.radius_sphere)
        return true; 
    return false;
}


bool intersect_sphere_cylinder(Sphere sph, Cylinder cyl)
{
    Vector cyl_axis_vector = {

        cyl.high_face_center.line - cyl.low_face_center.line,
        cyl.high_face_center.column - cyl.low_face_center.column,
        cyl.high_face_center.depth - cyl.low_face_center.depth
    };
    
    double length_axis = dist_two_points(cyl.low_face_center, cyl.high_face_center);
    
    if (are_equal(length_axis, 0))
    {
        double centerDist = dist_two_points(sph.center_sphere, cyl.low_face_center);
        
        if (centerDist <= (sph.radius_sphere + cyl.radius_cylinder))
            return true;
        return false;
    }
    Vector unit_axis_direction = vector_to_unit_vector(cyl_axis_vector);

    Vector w = {
        sph.center_sphere.line - cyl.low_face_center.line,
        sph.center_sphere.column - cyl.low_face_center.column,
        sph.center_sphere.depth - cyl.low_face_center.depth
    };

    double t = dot_product_vector(w, unit_axis_direction);

    Vector new_unit_axis;

    new_unit_axis = scalar_product_vector(t, unit_axis_direction);

    Vector q;

    q = diff_vector(w, new_unit_axis);

    double p = sqrt(magnitude_vector_squared(q));

    double shortest_distance;

    if (0 <= t && t <= length_axis)
    {
        if (p <= cyl.radius_cylinder)
            shortest_distance = 0;
        else
            shortest_distance = p - cyl.radius_cylinder;

    }
    else if (t < 0)
    {
        if (p <= cyl.radius_cylinder)
            shortest_distance = -t;
        else
            shortest_distance = sqrt((p - cyl.radius_cylinder)*(p - cyl.radius_cylinder) + t * t);
    }
    else
    {
        double s = t - length_axis;

        if (p <= cyl.radius_cylinder)
            shortest_distance = s;
        else
            shortest_distance = sqrt((p - cyl.radius_cylinder)*(p - cyl.radius_cylinder) + s * s);
    }

    double sep = shortest_distance - sph.radius_sphere;

    if (sep <= 0)
        return true;

    return false;
}


bool intersect_cylinder_cylinder(Cylinder cyl1, Cylinder cyl2)
{
    Vector a[3];

    int limit = 2;

    a[0] =
    {
        cyl1.axis_cylinder.pnt1.line - cyl1.axis_cylinder.pnt2.line,

        cyl1.axis_cylinder.pnt1.column - cyl1.axis_cylinder.pnt2.column,

        cyl1.axis_cylinder.pnt1.depth - cyl1.axis_cylinder.pnt2.depth
    };

    a[1] =
    {
        cyl2.axis_cylinder.pnt1.line - cyl2.axis_cylinder.pnt2.line,

        cyl2.axis_cylinder.pnt1.column - cyl2.axis_cylinder.pnt2.column,

        cyl2.axis_cylinder.pnt1.depth - cyl2.axis_cylinder.pnt2.depth
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
            cyl1.axis_cylinder.pnt1.line,

            cyl1.axis_cylinder.pnt1.column,
            
            cyl1.axis_cylinder.pnt1.depth
        };

        Vector p2 =
        {
            cyl1.axis_cylinder.pnt2.line,

            cyl1.axis_cylinder.pnt2.column,

            cyl1.axis_cylinder.pnt2.depth
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

        double r = cyl1.radius_cylinder * sqrt(1 - product * product);

        double min_cyl1 = std::min(p1_val, p2_val) - r;

        double max_cyl1 = std::max(p1_val, p2_val) + r;

        p1 =
        {
            cyl2.axis_cylinder.pnt1.line,

            cyl2.axis_cylinder.pnt1.column,

            cyl2.axis_cylinder.pnt1.depth
        };

        p2 =
        {
            cyl2.axis_cylinder.pnt2.line,

            cyl2.axis_cylinder.pnt2.column,

            cyl2.axis_cylinder.pnt2.depth
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

        r = cyl2.radius_cylinder * sqrt(1 - product * product);

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
