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
* @author     miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
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

