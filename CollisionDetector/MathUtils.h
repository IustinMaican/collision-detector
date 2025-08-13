#include <math.h>
#define EPSILON 0.000000001


#pragma region MATH_STRUCTURES
struct Vector
{
    double x;

    double y;

    double z;

    Vector& operator=(const Vector& other)
    {
        if (this != &other)
        {
            x = other.x;

            y = other.y;

            z = other.z;

        }

        return *this;
    }

};


struct point
{
    double linie;

    double coloana;

    double adancime;

    point& operator=(const point& other)
    {
        if (this != &other)
        {
            linie = other.linie;

            coloana = other.coloana;

            adancime = other.adancime;
        }

        return *this;
    }
};

std::istream& operator>>(std::istream& in, point& p)
{
    in >> p.linie;

    in >> p.coloana;

    in >> p.adancime;

    return in;
}

struct circle
{
    // sa modific sa tin doar cerc si tilt
    point center;

    point nord_point;

    point east_point;

    point south_point;

    point west_point;

    double radius;
};


struct segment
{
    point pnt1;

    point pnt2;
};


struct rectangle
{
    point colt_stanga_sus;

    point colt_stanga_jos;

    point colt_dreapta_sus;

    point colt_dreapta_jos;
};


class Sphere
{
public:

    point centru_sfera;

    double raza_sfera;

    void init()
    {
        std::cin >> centru_sfera;

        std::cin >> raza_sfera;
    }
};


class Parallelepiped
{
public:


    rectangle rectangle_sus;

    rectangle rectangle_jos;

    segment seg[12];

    rectangle faces_paral[6];

    void init()
    {
        std::cin >> rectangle_sus.colt_stanga_sus;

        std::cin >> rectangle_sus.colt_dreapta_sus;

        std::cin >> rectangle_sus.colt_stanga_jos;

        std::cin >> rectangle_sus.colt_dreapta_jos;

        std::cin >> rectangle_jos.colt_stanga_sus;

        std::cin >> rectangle_jos.colt_dreapta_sus;

        std::cin >> rectangle_jos.colt_stanga_jos;

        std::cin >> rectangle_jos.colt_dreapta_jos;

        seg[0] = { rectangle_sus.colt_stanga_sus, rectangle_sus.colt_dreapta_sus };

        seg[1] = { rectangle_sus.colt_stanga_jos, rectangle_sus.colt_dreapta_jos };

        seg[2] = { rectangle_jos.colt_stanga_sus, rectangle_jos.colt_dreapta_sus };

        seg[3] = { rectangle_jos.colt_stanga_jos, rectangle_jos.colt_dreapta_jos };

        seg[4] = { rectangle_sus.colt_stanga_sus, rectangle_jos.colt_stanga_sus };

        seg[5] = { rectangle_sus.colt_stanga_jos, rectangle_jos.colt_stanga_jos };

        seg[6] = { rectangle_sus.colt_dreapta_sus, rectangle_jos.colt_dreapta_sus };

        seg[7] = { rectangle_sus.colt_dreapta_jos, rectangle_jos.colt_dreapta_jos };

        seg[8] = { rectangle_sus.colt_stanga_sus, rectangle_sus.colt_stanga_jos };

        seg[9] = { rectangle_sus.colt_dreapta_sus, rectangle_sus.colt_dreapta_jos };

        seg[10] = { rectangle_jos.colt_dreapta_sus, rectangle_jos.colt_dreapta_jos };

        seg[11] = { rectangle_jos.colt_stanga_sus, rectangle_jos.colt_stanga_jos };

        faces_paral[0] = rectangle_jos;

        faces_paral[1] = rectangle_sus;

        faces_paral[2] = { rectangle_sus.colt_stanga_sus,  rectangle_jos.colt_stanga_sus,   rectangle_sus.colt_stanga_jos,  rectangle_jos.colt_stanga_jos };

        faces_paral[3] = { rectangle_sus.colt_dreapta_sus,  rectangle_jos.colt_dreapta_sus,   rectangle_sus.colt_dreapta_jos,  rectangle_jos.colt_dreapta_jos };

        faces_paral[4] = { rectangle_sus.colt_stanga_sus,  rectangle_jos.colt_stanga_sus,   rectangle_sus.colt_dreapta_sus,  rectangle_jos.colt_dreapta_sus };

        faces_paral[5] = { rectangle_sus.colt_stanga_jos,  rectangle_jos.colt_stanga_jos,   rectangle_sus.colt_dreapta_jos,  rectangle_jos.colt_dreapta_jos };
    }

};


class Cylinder
{
public:

    circle low_face;

    circle high_face;

    segment axa_Cylinder;

    double raza_Cylinder;

    void init()
    {
        std::cin >> low_face.center >> low_face.nord_point >> low_face.east_point >> low_face.south_point >> low_face.west_point;

        std::cin >> high_face.center >> high_face.nord_point >> high_face.east_point >> high_face.south_point >> high_face.west_point;

        std::cin >> raza_Cylinder;

        low_face.radius = raza_Cylinder;

        high_face.radius = raza_Cylinder;

        axa_Cylinder =
        {
            high_face.center,
            low_face.center
        };
    }

};

#pragma endregion MATH_STRUCTURES

struct solutions_of_equation
{
    double sol1;

    double sol2;

    solutions_of_equation& operator=(const solutions_of_equation& other)
    {
        if (this != &other)
        {
            sol1 = other.sol1;

            sol2 = other.sol2;

        }

        return *this;
    }
};

namespace MathUtils
{

#pragma region BASIC_MATH_UTILS

    bool are_equal(double nr1, double nr2);

    bool comp_points(point pnt1, point pnt2);

    double dist_two_points(point pnt1, point pnt2);

    double pythagoras(double length1, double length2);

#pragma endregion BASIC_MATH_UTILS

#pragma region VECTOR_UTILS

    Vector vector_to_unit_vector(Vector v);

    Vector cross_product_vector(Vector v1, Vector v2);

    Vector sum_vector(Vector v1, Vector v2);

    Vector diff_vector(Vector v1, Vector v2);

    Vector scalar_product_vector(double val, Vector v);

    double dot_product_vector(Vector v1, Vector v2);

    double magnitude_vector_squared(Vector v1);

#pragma endregion VECTOR_UTILS

    solutions_of_equation solve_equation_grade2(double a, double b, double c, bool& exista_pnt);
}