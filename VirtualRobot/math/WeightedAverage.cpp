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

#include "WeightedAverage.h"

using namespace math;



void math::WeightedFloatAverage::Add(float value, float weight)
{
    sum += value * weight;
    weightSum += weight;
}

float WeightedFloatAverage::Average()
{
    return sum / weightSum;
}

float WeightedFloatAverage::WeightSum()
{
    return weightSum;
}
void math::WeightedVec3Average::Add(Vec3 value, float weight)
{
    sum += value * weight;
    weightSum += weight;
}

Vec3 WeightedVec3Average::Average()
{
    return sum / weightSum;
}

float WeightedVec3Average::WeightSum()
{
    return weightSum;
}
