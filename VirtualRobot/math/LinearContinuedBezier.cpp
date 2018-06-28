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

#include "LinearContinuedBezier.h"

using namespace math;

math::LinearContinuedBezier::LinearContinuedBezier(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3)
    : p0(p0), p1(p1), p2(p2), p3(p3)
{
}

Eigen::Vector3f math::LinearContinuedBezier::Get(float t)
{
    if (0 <= t && t <= 1) return Bezier::CubicBezierPoint(p0, p1, p2, p3, t);
    if (t < 0) return p0 + (p0 - p1) * 3 * -t;
    return p3 + (p3 - p2) * 3 * (t - 1);
}

Eigen::Vector3f math::LinearContinuedBezier::GetDerivative(float t)
{
    if (0 <= t && t <= 1) return Bezier::CubicBezierDerivative(p0, p1, p2, p3, t);
    if (t < 0) return (p0 - p1) * -3;
    return (p3 - p2) * 3;
}
