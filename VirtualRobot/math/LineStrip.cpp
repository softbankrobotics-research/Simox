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
* @author     miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "LineStrip.h"
#include "Helpers.h"

using namespace math;



LineStrip::LineStrip(Vec3ListPtr points, float minT, float maxT)
{
    this->points = points;
    this->minT = minT;
    this->maxT = maxT;
}


Eigen::Vector3f LineStrip::Get(float t)
{
    int i; float f;
    GetIndex(t,  i,  f);
    return points->at(i) * (1 - f) + points->at(i+1) * f;
}

Eigen::Vector3f LineStrip::GetDirection(int i)
{
    return points->at(i+1) - points->at(i);
}

void LineStrip::GetIndex(float t, int &i, float &f)
{
    Helpers::GetIndex(t, minT, maxT, points->size(),  i,  f);
}

