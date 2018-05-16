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

    struct WeightedFloatAverage
    {
    public:
        void Add(float value, float weight);
        float Average();
        float WeightSum();
    private:
        float sum = 0;
        float weightSum = 0;
    };

    class WeightedVec3Average
    {
    public:
        void Add(Eigen::Vector3f value, float weight);
        Eigen::Vector3f Average();
        float WeightSum();
    private:
        Eigen::Vector3f sum;
        float weightSum = 0;
    };
}

