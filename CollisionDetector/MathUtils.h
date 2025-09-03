#include <math.h>

constexpr double EPSILON = 1e-9;


#pragma region MATH_STRUCTURES
struct Vector
{
    double x;

    double y;

    double z;
};


struct point
{
    double line;

    double column;

    double depth;
};

std::istream& operator>>(std::istream& in, point& p)
{
    in >> p.line;

    in >> p.column;

    in >> p.depth;

    return in;
}

std::ostream& operator<<(std::ostream& out, const point& p)
{
    out << p.line << " " << p.column << " " << p.depth;

    return out;
}

struct segment
{
    point pnt1;

    point pnt2;
};


class Sphere
{
public:

    point center_sphere;

    double radius_sphere;

    void init()
    {
        std::cin >> center_sphere;

        std::cin >> radius_sphere;
    }
};


class Parallelepiped
{
public:

    point P;
    // u line, v column, w adanmcime
    Vector u, v, w;
    // x line, y column, z depth
    double hx, hy, hz;
};


class Cylinder
{
public:

    point low_face_center;

    point high_face_center;

    segment axis_cylinder;

    double radius_cylinder;

    void init()
    {
        std::cin >> low_face_center;

        std::cin >> high_face_center;

        std::cin >> radius_cylinder;

        axis_cylinder =
        {
            high_face_center,
            low_face_center
        };
    }

};

#pragma endregion MATH_STRUCTURES

namespace MathUtils
{

#pragma region BASIC_MATH_UTILS

    bool are_equal(double nr1, double nr2);

    double dist_two_points(const point& pnt1, const point& pnt2);

    double pythagoras(double length1, double length2);

#pragma endregion BASIC_MATH_UTILS

#pragma region VECTOR_UTILS

    Vector vector_to_unit_vector(const Vector& v);

    Vector cross_product_vector(const Vector& v1, const Vector& v2);

    Vector diff_vector(const Vector& v1, const Vector& v2);

    Vector scalar_product_vector(double val, const Vector& v);

    double dot_product_vector(const Vector& v1, const Vector& v2);

    double magnitude_vector_squared(const Vector& v1);

#pragma endregion VECTOR_UTILS

}
