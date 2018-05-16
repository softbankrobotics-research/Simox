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
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "MathForwardDefinitions.h"


namespace math
{

    class Triangle
    {
    public:
        Triangle();//TODO
        Triangle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

        Eigen::Vector3f P1() const {return p1;}
        Eigen::Vector3f P2() const {return p2;}
        Eigen::Vector3f P3() const {return p3;}

        std::string ToString();

        Eigen::Vector3f Normal() const;

        Triangle Flipped();

        Eigen::Vector3f Centroid();
        std::vector<Triangle> Subdivide(int depth);

        static void Subdivide(int depth, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, std::vector<Triangle> &list);

        Triangle Transform(Eigen::Vector3f center, float unitLength);
    private:
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Eigen::Vector3f p3;
    };
}

