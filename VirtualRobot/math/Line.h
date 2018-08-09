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

#include "AbstractFunctionR1R3.h"
#include "MathForwardDefinitions.h"


namespace math
{

    class Line
            : public AbstractFunctionR1R3
    {
    public:
        Line(Eigen::Vector3f pos, Eigen::Vector3f dir);
        Eigen::Vector3f Pos(){return pos;}
        Eigen::Vector3f Dir(){return dir;}

        Line Normalized();
        Eigen::Vector3f Get(float t) override;
        Eigen::Vector3f GetDerivative(float t) override;
        Eigen::Vector3f GetClosestPoint(Eigen::Vector3f p);
        float GetT(Eigen::Vector3f p);
        std::string ToString();

        bool IntersectsTriangle(Triangle tri, float& t);
        bool IntersectsPrimitive(PrimitivePtr p, float& t);

        static Line FromPoints(Eigen::Vector3f p1, Eigen::Vector3f p2);
        static Line FromPoses(const Eigen::Matrix4f& p1, const Eigen::Matrix4f& p2);

        Line Transform(const Eigen::Matrix4f& pose);

    private:
        Eigen::Vector3f pos;
        Eigen::Vector3f dir;
    };
}

