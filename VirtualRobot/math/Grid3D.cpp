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

#include "Grid3D.h"
#include "cmath"
#include "Helpers.h"

using namespace math;


Grid3D::Grid3D(Vec3 p1, Vec3 p2, int stepsX, int stepsY, int stepsZ)
{
    this->p1 = p1;
    this->p2 = p2;
    this->stepsX = stepsX;
    this->stepsY = stepsY;
    this->stepsZ = stepsZ;
}

Grid3DPtr Grid3D::CreateFromBox(Vec3 p1, Vec3 p2, float stepLength)
{
    Vec3 steps = (p2 - p1) / stepLength;
    return Grid3DPtr(new  Grid3D(p1, p2, std::round(steps.x()), std::round(steps.y()), std::round(steps.z())));
}


Vec3 Grid3D::Get(int x, int y, int z){
    return Vec3(Helpers::Lerp(p1.x(), p2.x(), 0, stepsX, x),
                Helpers::Lerp(p1.y(), p2.y(), 0, stepsY, y),
                Helpers::Lerp(p1.z(), p2.z(), 0, stepsZ, z));
}

std::vector<Vec3> Grid3D::AllGridPoints()
{
    std::vector<Vec3> points;
    for (int x = 0; x <= stepsX; x++)
    {
        for (int y = 0; y <= stepsY; y++)
        {
            for (int z = 0; z <= stepsZ; z++)
            {
                points.push_back(Get(x, y, z));
            }
        }
    }
    return points;
}
