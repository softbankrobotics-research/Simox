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

    class DataR3R2
    {
    public:
        Eigen::Vector3f Position() const { return position; }
        const Eigen::Vector2f& Value() const { return value; }
        float Value1() const { return value(0); }
        float Value2() const { return value(1); }
        DataR3R2(Eigen::Vector3f position, float value1, float value2);
        DataR3R2(float x, float y, float z, float value1, float value2);
        std::string ToString();
    private:
        Eigen::Vector3f position;
        Eigen::Vector2f value;
    };
}

