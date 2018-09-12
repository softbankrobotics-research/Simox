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


#include "../VirtualRobot.h"
#include "SimpleAbstractFunctionR3R1.h"


namespace math
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT AbstractFunctionR3R1 :
            public SimpleAbstractFunctionR3R1
    {
    public:
       virtual float GetDdx(Eigen::Vector3f pos) = 0;
       virtual float GetDdy(Eigen::Vector3f pos) = 0;
       virtual float GetDdz(Eigen::Vector3f pos) = 0;

    private:
    };
}

