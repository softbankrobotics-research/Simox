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

#include "Plane.h"
#include "Helpers.h"

using namespace math;

Plane::Plane(Eigen::Vector3f pos, Eigen::Vector3f dir1, Eigen::Vector3f dir2)
    : pos(pos), dir1(dir1), dir2(dir2)
{
}

Eigen::Vector3f Plane::GetPoint(float u, float v)
{
    return pos + u * dir1 + v * dir2;
}

Eigen::Vector3f Plane::GetDdu(float, float)
{
    return dir1;
}

Eigen::Vector3f Plane::GetDdv(float, float)
{
    return dir2;
}

void Plane::GetUV(Eigen::Vector3f pos, float& u, float& v)
{
    float w;
    GetUVW(pos,u,v,w);
}

Eigen::Vector3f Plane::GetNormal()
{
    return dir1.cross(dir2);
}

Plane Plane::SwappedDirections()
{
    return Plane(pos, dir2, dir1);
}

Plane Plane::Normalized()
{
    return Plane(pos, dir1.normalized(), dir2.normalized());
}

ImplicitPlane Plane::ToImplicit()
{
    return ImplicitPlane::FromPositionNormal(pos, GetNormal());
}

Eigen::Matrix3f Plane::GetRotationMatrix()
{
    Helpers::AssertNormalized(dir1);
    Helpers::AssertNormalized(dir2);
    Eigen::Matrix3f result;
    result << dir1 , dir2 , GetNormal();
    return result;
}

std::string Plane::ToString()
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir1 << ")" << ") (" << dir2 << ")";
    return ss.str();
}

void Plane::GetUVW(Eigen::Vector3f pos, float& u, float& v, float& w)
{
    Eigen::Vector3f rotated = GetRotationMatrix().transpose() * (pos - this->pos);
    u = rotated.x();
    v = rotated.y();
    w = rotated.z();

}

Plane Plane::FromNormal(Eigen::Vector3f pos, Eigen::Vector3f normal)
{
    Eigen::Vector3f d1, d2;
    Helpers::GetOrthonormalVectors(normal, d1,d2);
    return Plane(pos, d1, d2);
}

math::Plane math::Plane::Transform(const Eigen::Matrix4f &transform)
{
    return Plane(Helpers::TransformPosition(transform, pos), Helpers::TransformDirection(transform, dir1), Helpers::TransformDirection(transform, dir2));
}
