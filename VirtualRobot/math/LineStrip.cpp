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

#include "LineStrip.h"
#include "Helpers.h"

using namespace math;



LineStrip::LineStrip(const std::vector<Eigen::Vector3f> &points, float minT, float maxT)
    : points(points), minT(minT), maxT(maxT)
{
}

bool LineStrip::InLimits(float t)
{
    return t >= minT && t <= maxT;
}

Eigen::Vector3f LineStrip::Get(float t)
{
    int i; float f;
    GetIndex(t,  i,  f);
    return points.at(i) * (1 - f) + points.at(i+1) * f;
}

Eigen::Vector3f LineStrip::GetDirection(int i)
{
    return points.at(i+1) - points.at(i);
}

void LineStrip::GetIndex(float t, int &i, float &f)
{
    Helpers::GetIndex(t, minT, maxT, points.size(),  i,  f);
}

