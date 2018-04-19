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

#pragma once

#include "MathForwardDefinitions.h"

#include "AbstractFunctionR1R3.h"

namespace math
{

    class Bezier :
            public AbstractFunctionR1R3
    {
    public:
        Bezier(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

        Eigen::Vector3f P0(){return p0;}
        Eigen::Vector3f P1(){return p1;}
        Eigen::Vector3f P2(){return p2;}
        Eigen::Vector3f P3(){return p3;}

        Eigen::Vector3f Get(float t) override;
        Eigen::Vector3f GetDerivative(float t);
        static Eigen::Vector3f CubicBezierPoint(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, float t);
        static std::vector<Eigen::Vector3f> CubicBezier(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, int steps);
        static Eigen::Vector3f CubicBezierDerivative(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, float t);

    private:
        Eigen::Vector3f p0;
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Eigen::Vector3f p3;

        static float Pow3(float x);
        static float Pow2(float x);
    };
}

