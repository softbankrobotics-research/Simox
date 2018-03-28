/*
* This file is part of ArmarX.
*
* ArmarX is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* ArmarX is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "Plane.h"
#include "Helpers.h"

using namespace math;

Plane::Plane(Vec3 pos, Vec3 dir1, Vec3 dir2)
    : pos(pos), dir1(dir1), dir2(dir2)
{
}

Vec3 Plane::GetPoint(float u, float v)
{
    return pos + u * dir1 + v * dir2;
}

Vec3 Plane::GetDdu(float, float)
{
    return dir1;
}

Vec3 Plane::GetDdv(float, float)
{
    return dir2;
}

void Plane::GetUV(Vec3 pos, float& u, float& v)
{
    float w;
    GetUVW(pos,u,v,w);
}

Vec3 Plane::GetNormal()
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

Matrix3f Plane::GetRotationMatrix()
{
    Helpers::AssertNormalized(dir1);
    Helpers::AssertNormalized(dir2);
    Matrix3f result;
    result << dir1 , dir2 , GetNormal();
    return result;
}

std::string Plane::ToString()
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir1 << ")" << ") (" << dir2 << ")";
    return ss.str();
}

void Plane::GetUVW(Vec3 pos, float& u, float& v, float& w)
{
    Vec3 rotated = GetRotationMatrix().transpose() * (pos - this->pos);
    u = rotated.x();
    v = rotated.y();
    w = rotated.z();

}

Plane Plane::FromNormal(Vec3 pos, Vec3 normal)
{
    Vec3 d1, d2;
    Helpers::GetOrthonormalVectors(normal, d1,d2);
    return Plane(pos, d1, d2);
}




