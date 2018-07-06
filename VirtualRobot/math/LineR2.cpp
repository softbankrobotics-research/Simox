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

#include "LineR2.h"


using namespace math;



LineR2::LineR2(Eigen::Vector3f pos, Eigen::Vector3f dir)
    : pos(pos), dir(dir)
{
}

LineR2 LineR2::Normalized()
{
    return LineR2(pos,dir.normalized());
}

Eigen::Vector3f LineR2::Get(float t)
{
    return pos + t * dir;
}

Eigen::Vector3f LineR2::GetDerivative(float)
{
    return dir;
}

Eigen::Vector3f LineR2::GetClosestPoint(Eigen::Vector3f p)
{
    return pos - (pos - p).dot(dir) * dir / dir.squaredNorm();
}

float LineR2::GetT(Eigen::Vector3f p)
{
    return (p - pos).dot(dir) / dir.squaredNorm();

}

std::string LineR2::ToString()
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir << ")";
    return ss.str();

}

//bool LineR2::Intersect(const Eigen::Vector3f &pos1, const Eigen::Vector3f &dir1, const Eigen::Vector3f &pos2, const Eigen::Vector3f &dir2, float &t, float &u)
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
