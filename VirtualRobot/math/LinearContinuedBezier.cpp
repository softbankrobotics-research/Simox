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

#include "LinearContinuedBezier.h"

using namespace math;

math::LinearContinuedBezier::LinearContinuedBezier(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
    : p0(p0), p1(p1), p2(p2), p3(p3)
{
}

Vec3 math::LinearContinuedBezier::Get(float t)
{
    if (0 <= t && t <= 1) return Bezier::CubicBezierPoint(p0, p1, p2, p3, t);
    if (t < 0) return p0 + (p0 - p1) * 3 * -t;
    return p3 + (p3 - p2) * 3 * (t - 1);
}

Vec3 math::LinearContinuedBezier::GetDerivative(float t)
{
    if (0 <= t && t <= 1) return Bezier::CubicBezierDerivative(p0, p1, p2, p3, t);
    if (t < 0) return (p0 - p1) * -3;
    return (p3 - p2) * 3;
}
