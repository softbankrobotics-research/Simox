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

#ifndef math_HapticExplorationLibrary_ImplicitPlane
#define math_HapticExplorationLibrary_ImplicitPlane

#include "MathForwardDefinitions.h"

#include "AbstractFunctionR3R1.h"




namespace math
{

class ImplicitPlane
        : public SimpleAbstractFunctionR3R1
{
public:
    // ax + by + cz = d
    ImplicitPlane(float a, float b, float c, float d);

    float A() { return a; }
    float B() { return b; }
    float C() { return c; }
    float D() { return d; }

    ImplicitPlane Normalize();
    ImplicitPlane Flipped();
    Vec3 GetNormal();
    Vec3 GetClosestPoint(Vec3 v);
    static ImplicitPlane FromPositionNormal(Vec3 pos, Vec3 normal);
    static ImplicitPlane FromContact(Contact c);

    // https://de.wikipedia.org/wiki/Schnittgerade
    Line Intersect(Plane plane);

    float Get(Vec3 pos);

private:
    float a;
    float b;
    float c;
    float d;

};
}

#endif // math_HapticExplorationLibrary_ImplicitPlane
