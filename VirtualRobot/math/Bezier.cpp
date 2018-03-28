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
math::Bezier::Bezier(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
    : p0(p0), p1(p1), p2(p2), p3(p3)
{
}

Vec3 math::Bezier::Get(float t)
{
    return CubicBezierPoint(p0, p1, p2, p3, t);
}

Vec3 math::Bezier::GetDerivative(float t)
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

Vec3 math::Bezier::CubicBezierPoint(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, float t)
{
    return Pow3(1 - t) * p0 + 3 * t * Pow2(1 - t) * p1 + 3 * Pow2(t) * (1 - t) * p2 + Pow3(t) * p3;
}

std::vector<Vec3> math::Bezier::CubicBezier(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, int steps)
{
    std::vector<Vec3> points;
    for (int i = 0; i <= steps; i++)
    {
       points.push_back(CubicBezierPoint(p0, p1, p2, p3, 1.f / steps * i));
    }
    return points;
}

Vec3 math::Bezier::CubicBezierDerivative(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, float t)
{
    return -3 * Pow2(1 - t) * p0 + 3 * Pow2(1 - t) * p1 - 6 * (1 - t) * t * p1 + 6 * (1 - t) * t * p2 - 3 * Pow2(t) * p2 + 3 * Pow2(t) * p3;
}
