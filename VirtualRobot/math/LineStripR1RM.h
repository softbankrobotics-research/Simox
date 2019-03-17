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
 * @author     Raphael Grimm (raphael dot grimm at kit dot edu)
 * @copyright  2018 Raphael Grimm
 *             GNU Lesser General Public License
 */

#pragma once
#include "MathForwardDefinitions.h"
#include "Helpers.h"
#include "SimpleAbstractFunctionRNRM.h"


namespace math
{
    template<int M>
    class LineStripR1RM
        : public SimpleAbstractFunctionR1RM<M>
    {
    public:
        using point_t = Eigen::Matrix<float, M, 1>;

        int Count() const
        {
            return points.cols();
        }

        LineStripR1RM(Eigen::Matrix < float, M, -1 > points, float minT, float maxT)
            : points(std::move(points)), minT(minT), maxT(maxT)
        {}

        LineStripR1RM(std::vector<point_t> pointv, float minT, float maxT)
            : points(M, pointv.size()), minT(minT), maxT(maxT)
        {
            for (int i = 0; i < Count(); ++i)
            {
                points.col(i) = pointv.at(i);
            }
        }

        LineStripR1RM(LineStripR1RM&&) = default;
        LineStripR1RM(const LineStripR1RM&) = default;

        LineStripR1RM& operator=(LineStripR1RM&&) = default;
        LineStripR1RM& operator=(const LineStripR1RM&) = default;

        bool InLimits(float t) const
        {
            return t >= minT && t <= maxT;
        }
        point_t Get(float t) const override
        {
            int i;
            float f;
            GetIndex(t,  i,  f);
            return points.col(i) * (1 - f) + points.col(i + 1) * f;
        }

    private:
        point_t GetDirection(int i) const
        {
            return points.col(i + 1) - points.col(i);
        }
        void GetIndex(float t,  int& i, float& f) const
        {
            Helpers::GetIndex(t, minT, maxT, Count(),  i,  f);
        }

        Eigen::Matrix < float, M, -1 > points;
        float minT, maxT;
    };
    using LineStripR1R1 = LineStripR1RM<1>;
    using LineStripR1R2 = LineStripR1RM<2>;
    using LineStripR1R3 = LineStripR1RM<3>;
    using LineStripR1R4 = LineStripR1RM<4>;
    using LineStripR1R5 = LineStripR1RM<5>;
    using LineStripR1R6 = LineStripR1RM<6>;
    using LineStripR1RX = LineStripR1RM < -1 >;
}

