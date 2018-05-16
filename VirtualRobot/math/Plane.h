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
#include "AbstractFunctionR2R3.h"
#include "Line.h"
#include "ImplicitPlane.h"





namespace math
{

class Plane :
        public AbstractFunctionR2R3
{

public:
    Plane(Eigen::Vector3f pos, Eigen::Vector3f dir1, Eigen::Vector3f dir2);


    Eigen::Vector3f Pos() {return pos;}
    Eigen::Vector3f Dir1(){return dir1;}
    Eigen::Vector3f Dir2(){return dir2;}


    Eigen::Vector3f GetPoint(float u, float v) override;
    Eigen::Vector3f GetDdu(float u, float v) override;
    Eigen::Vector3f GetDdv(float u, float v) override;
    void GetUV(Eigen::Vector3f pos, float &u, float &v) override;
    Eigen::Vector3f GetNormal();
    Plane SwappedDirections();
    Plane Normalized();
    ImplicitPlane ToImplicit();
    Eigen::Matrix3f GetRotationMatrix();
    std::string ToString();
    void GetUVW(Eigen::Vector3f pos, float &u, float &v, float &w);
    float GetW(Eigen::Vector3f pos);
    float Intersect(Line line, bool* exists);
    Line Intersect(Plane p2);

    static Plane FromNormal(Eigen::Vector3f pos, Eigen::Vector3f normal);
private:
    Eigen::Vector3f pos;
    Eigen::Vector3f dir1;
    Eigen::Vector3f dir2;

};
}

