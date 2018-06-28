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





namespace math
{

class LineR2
{
public:
    LineR2(Eigen::Vector3f pos, Eigen::Vector3f dir);

    Eigen::Vector3f Pos(){return pos;}
    Eigen::Vector3f Dir(){return dir;}

    LineR2 Normalized ();
    Eigen::Vector3f Get(float t);
    Eigen::Vector3f GetDerivative(float t);
    Eigen::Vector3f GetClosestPoint(Eigen::Vector3f p);
    float GetT(Eigen::Vector3f p);
    std::string ToString();
    //Vec3Opt Intersect(const LineR2& l2);
    //static bool Intersect(const Eigen::Vector3f& pos1, const Eigen::Vector3f& dir1, const Eigen::Vector3f& pos2, const Eigen::Vector3f& dir2, float& t,float& u);

    //static bool Intersect(LineR2 l1, LineR2 l2, out float t, out float u);


private:
    Eigen::Vector3f pos;
    Eigen::Vector3f dir;

};
}

