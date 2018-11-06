/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#include "ImplicitPlane.h"
#include "Contact.h"
#include "Plane.h"
using namespace math;


ImplicitPlane::ImplicitPlane(float a, float b, float c, float d)
    : a(a), b(b), c(c), d(d)
{
}

ImplicitPlane ImplicitPlane::Normalize()
{
    float len = GetNormal().norm();
    return ImplicitPlane(a / len, b / len, c / len, d / len);
}

ImplicitPlane ImplicitPlane::Flipped()
{
     return ImplicitPlane(-a, -b, -c, -d);
}

Eigen::Vector3f ImplicitPlane::GetNormal()
{
    return Eigen::Vector3f(a, b, c);
}

Eigen::Vector3f ImplicitPlane::GetClosestPoint(Eigen::Vector3f v)
{
    Eigen::Vector3f normal = GetNormal();
    float d = this->d - v.dot(normal);
    float denominator = a * a + b * b + c * c;
    return normal * d / denominator + v;

}

ImplicitPlane ImplicitPlane::FromPositionNormal(Eigen::Vector3f pos, Eigen::Vector3f normal)
{
    return ImplicitPlane(normal.x(), normal.y(), normal.z(), normal.dot(pos));

}

ImplicitPlane ImplicitPlane::FromContact(Contact c)
{
    return FromPositionNormal(c.Position(), c.Normal());
}

float ImplicitPlane::GetSignedDistance(const Eigen::Vector3f &p)
{
    return GetNormal().dot(p) + d;
}

Line ImplicitPlane::Intersect(Plane plane)
{

        Eigen::Vector3f n = GetNormal();
        Eigen::Vector3f p = plane.Pos();
        Eigen::Vector3f u = plane.Dir1();
        Eigen::Vector3f v = plane.Dir2();
        if (std::abs(n.dot(u)) < std::abs(n.dot(v)))
        {
            Eigen::Vector3f tmp = u;
            u = v;
            v = tmp;
        }

        Eigen::Vector3f pos = p + (d - n.dot(p)) / n.dot(u) * u;
        Eigen::Vector3f dir = v - n.dot(v) / n.dot(u) * u;
        return Line(pos, dir);

}


float ImplicitPlane::Get(Eigen::Vector3f pos)
{
 return GetNormal().dot(pos - GetClosestPoint(pos));
}
