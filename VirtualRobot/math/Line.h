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

#ifndef math_HapticExplorationLibrary_Line
#define math_HapticExplorationLibrary_Line

#include "MathForwardDefinitions.h"


namespace math
{

    class Line
    {
    public:
        Line(Vec3 pos, Vec3 dir);
        Vec3 Pos(){return pos;}
        Vec3 Dir(){return dir;}

        Line Normalized();
        Vec3 Get(float t);
        Vec3 GetDerivative(float t);
        Vec3 GetClosestPoint(Vec3 p);
        float GetT(Vec3 p);
        std::string ToString();

        bool IntersectsTriangle(Triangle tri, float& t);
        bool IntersectsPrimitive(PrimitivePtr p, float& t);

        static Line FromPoints(Vec3 p1, Vec3 p2);

    private:
        Vec3 pos;
        Vec3 dir;
    };
}

#endif // math_HapticExplorationLibrary_Line
