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

#include "../VirtualRobot.h"
#include "MathForwardDefinitions.h"

namespace math
{
    template<int N, int M>
    class VIRTUAL_ROBOT_IMPORT_EXPORT SimpleAbstractFunctionRNRM
    {
    public:
        using in_t = Eigen::Matrix<float, N, 1>;
        using out_t = Eigen::Matrix<float, M, 1>;
        virtual out_t Get(in_t t) const = 0;
    private:
    };

    template<int M>
    class VIRTUAL_ROBOT_IMPORT_EXPORT SimpleAbstractFunctionRNRM<1, M>
    {
    public:
        using in_t = float;
        using out_t = Eigen::Matrix<float, M, 1>;
        virtual out_t Get(in_t t) const = 0;
    private:
    };

    template<int M>
    using SimpleAbstractFunctionR1RM = SimpleAbstractFunctionRNRM<1, M>;
}

