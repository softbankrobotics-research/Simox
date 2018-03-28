#include "Triangle.h"

using namespace math;


Triangle::Triangle()
    : p1(Vec3(0,0,0)), p2(Vec3(0,0,0)), p3(Vec3(0,0,0))
{
}

Triangle::Triangle(Vec3 p1, Vec3 p2, Vec3 p3)
    : p1(p1), p2(p2), p3(p3)
{
}

std::string Triangle::ToString()
{
    std::stringstream ss;
    ss << "(" << p1 << ") (" << p2 << ") (" << p3 << ")";
    return ss.str();
}

Vec3 Triangle::Normal() const
{
    return (p2 - p1).cross(p3 - p1);
}

Triangle Triangle::Flipped()
{
    return Triangle(p2, p1, p3);
}

Vec3 Triangle::Centroid()
{
    return (p1 + p2 + p3) / 3;
}

std::vector<Triangle> Triangle::Subdivide(int depth)
{
    std::vector<Triangle> list;
    Subdivide(depth, p1, p2, p3, list);
    return list;
}

void Triangle::Subdivide(int depth, Vec3 p1, Vec3 p2, Vec3 p3, std::vector<Triangle> &list)
{
    if (depth <= 0)
    {
        list.push_back(Triangle(p1, p2, p3));
        return;
    }
    Vec3 c = (p1 + p2 + p3) / 3;
    Subdivide(depth - 1, p1, p2, c, list);
    Subdivide(depth - 1, p2, p3, c, list);
    Subdivide(depth - 1, p3, p1, c, list);
}

Triangle Triangle::Transform(Vec3 center, float unitLength)
{
      return Triangle(
                    p1 * unitLength + center,
                    p2 * unitLength + center,
                    p3 * unitLength + center);
}







