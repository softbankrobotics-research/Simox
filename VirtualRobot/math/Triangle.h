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

#ifndef math_Triangle
#define math_Triangle

#include "MathForwardDefinitions.h"


namespace math
{

    class Triangle
    {
    public:
        Triangle();//TODO
        Triangle(Vec3 p1, Vec3 p2, Vec3 p3);

        Vec3 P1() const {return p1;}
        Vec3 P2() const {return p2;}
        Vec3 P3() const {return p3;}

        std::string ToString();

        Vec3 Normal() const;

        Triangle Flipped();

        Vec3 Centroid();
        std::vector<Triangle> Subdivide(int depth);

        static void Subdivide(int depth, Vec3 p1, Vec3 p2, Vec3 p3, std::vector<Triangle> &list);

        Triangle Transform(Vec3 center, float unitLength);
    private:
        Vec3 p1;
        Vec3 p2;
        Vec3 p3;
    };
}

#endif //math_Triangle
