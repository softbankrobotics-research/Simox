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

#include "LineR2.h"


using namespace math;



LineR2::LineR2(Vec3 pos, Vec3 dir)
    : pos(pos), dir(dir)
{
}

LineR2 LineR2::Normalized()
{
    return LineR2(pos,dir.normalized());
}

Vec3 LineR2::Get(float t)
{
    return pos + t * dir;
}

Vec3 LineR2::GetDerivative(float)
{
    return dir;
}

Vec3 LineR2::GetClosestPoint(Vec3 p)
{
    return pos - (pos - p).dot(dir) * dir / dir.squaredNorm();
}

float LineR2::GetT(Vec3 p)
{
    return (p - pos).dot(dir) / dir.squaredNorm();

}

std::string LineR2::ToString()
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir << ")";
    return ss.str();

}

//bool LineR2::Intersect(const Vec3 &pos1, const Vec3 &dir1, const Vec3 &pos2, const Vec3 &dir2, float &t, float &u)
//{
//
//}
//
//Vec3Opt LineR2::Intersect(const LineR2& l2)
//{
//    float t, u;
//    if (Intersect(this, l2, &t, &u))
//    {
//        return Vec3Opt(Get(t));
//    }
//    else
//    {
//        return Vec3Opt();
//    }
//}
