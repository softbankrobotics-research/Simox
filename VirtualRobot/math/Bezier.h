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

