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

#ifndef math_Bezier
#define math_Bezier

#include "MathForwardDefinitions.h"

#include "AbstractFunctionR1R3.h"

namespace math
{

    class Bezier :
            public AbstractFunctionR1R3
    {
    public:
        Bezier(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3);

        Vec3 P0(){return p0;}
        Vec3 P1(){return p1;}
        Vec3 P2(){return p2;}
        Vec3 P3(){return p3;}

        Vec3 Get(float t) override;
        Vec3 GetDerivative(float t);
        static Vec3 CubicBezierPoint(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, float t);
        static std::vector<Vec3> CubicBezier(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, int steps);
        static Vec3 CubicBezierDerivative(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, float t);

    private:
        Vec3 p0;
        Vec3 p1;
        Vec3 p2;
        Vec3 p3;

        static float Pow3(float x);
        static float Pow2(float x);
    };
}

#endif //math_Bezier
