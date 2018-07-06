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

#include "SimpleAbstractFunctionR1R3.h"


namespace math
{

    class LineStrip
            : public SimpleAbstractFunctionR1R3
    {
    public:
        Vec3ListPtr points;
        float minT, maxT;

        int Count() {  return points->size();  }

        LineStrip(Vec3ListPtr points, float minT, float maxT);

        bool InLimits(float t);
         Eigen::Vector3f Get(float t) override;

    private:
        Eigen::Vector3f GetDirection(int i);
        void GetIndex(float t,  int& i, float& f);

    };
}

