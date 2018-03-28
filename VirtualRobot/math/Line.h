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
        Line(Eigen::Vector3f pos, Eigen::Vector3f dir);
        Eigen::Vector3f Pos(){return pos;}
        Eigen::Vector3f Dir(){return dir;}

        Line Normalized();
        Eigen::Vector3f Get(float t);
        Eigen::Vector3f GetDerivative(float t);
        Eigen::Vector3f GetClosestPoint(Eigen::Vector3f p);
        float GetT(Eigen::Vector3f p);
        std::string ToString();

        bool IntersectsTriangle(Triangle tri, float& t);
        bool IntersectsPrimitive(PrimitivePtr p, float& t);

        static Line FromPoints(Eigen::Vector3f p1, Eigen::Vector3f p2);

    private:
        Eigen::Vector3f pos;
        Eigen::Vector3f dir;
    };
}

#endif // math_HapticExplorationLibrary_Line
