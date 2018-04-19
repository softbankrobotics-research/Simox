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

#include "Bezier.h"

using namespace math;
math::Bezier::Bezier(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3)
    : p0(p0), p1(p1), p2(p2), p3(p3)
{
}

Eigen::Vector3f math::Bezier::Get(float t)
{
    return CubicBezierPoint(p0, p1, p2, p3, t);
}

Eigen::Vector3f math::Bezier::GetDerivative(float t)
{
    return CubicBezierDerivative(p0, p1, p2, p3, t);
}

float math::Bezier::Pow3(float x)
{
    return x * x * x;
}

float math::Bezier::Pow2(float x)
{
    return x * x;
}

Eigen::Vector3f math::Bezier::CubicBezierPoint(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, float t)
{
    return Pow3(1 - t) * p0 + 3 * t * Pow2(1 - t) * p1 + 3 * Pow2(t) * (1 - t) * p2 + Pow3(t) * p3;
}

std::vector<Eigen::Vector3f> math::Bezier::CubicBezier(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, int steps)
{
    std::vector<Eigen::Vector3f> points;
    for (int i = 0; i <= steps; i++)
    {
       points.push_back(CubicBezierPoint(p0, p1, p2, p3, 1.f / steps * i));
    }
    return points;
}

Eigen::Vector3f math::Bezier::CubicBezierDerivative(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, float t)
{
    return -3 * Pow2(1 - t) * p0 + 3 * Pow2(1 - t) * p1 - 6 * (1 - t) * t * p1 + 6 * (1 - t) * t * p2 - 3 * Pow2(t) * p2 + 3 * Pow2(t) * p3;
}
