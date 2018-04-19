#include "Triangle.h"

using namespace math;


Triangle::Triangle()
    : p1(Eigen::Vector3f(0,0,0)), p2(Eigen::Vector3f(0,0,0)), p3(Eigen::Vector3f(0,0,0))
{
}

Triangle::Triangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3)
    : p1(p1), p2(p2), p3(p3)
{
}

std::string Triangle::ToString()
{
    std::stringstream ss;
    ss << "(" << p1 << ") (" << p2 << ") (" << p3 << ")";
    return ss.str();
}

Eigen::Vector3f Triangle::Normal() const
{
    return (p2 - p1).cross(p3 - p1);
}

Triangle Triangle::Flipped()
{
    return Triangle(p2, p1, p3);
}

Eigen::Vector3f Triangle::Centroid()
{
    return (p1 + p2 + p3) / 3;
}

std::vector<Triangle> Triangle::Subdivide(int depth)
{
    std::vector<Triangle> list;
    Subdivide(depth, p1, p2, p3, list);
    return list;
}

void Triangle::Subdivide(int depth, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, std::vector<Triangle> &list)
{
    if (depth <= 0)
    {
        list.push_back(Triangle(p1, p2, p3));
        return;
    }
    Eigen::Vector3f c = (p1 + p2 + p3) / 3;
    Subdivide(depth - 1, p1, p2, c, list);
    Subdivide(depth - 1, p2, p3, c, list);
    Subdivide(depth - 1, p3, p1, c, list);
}

Triangle Triangle::Transform(Eigen::Vector3f center, float unitLength)
{
      return Triangle(
                    p1 * unitLength + center,
                    p2 * unitLength + center,
                    p3 * unitLength + center);
}







