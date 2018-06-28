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
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "DataR3R2.h"

using namespace math;


DataR3R2::DataR3R2(Eigen::Vector3f position, float value1, float value2)
    : position(position), value(value1, value2)
{
}

DataR3R2::DataR3R2(float x, float y, float z, float value1, float value2)
    : position(x, y, z), value(value1, value2)
{

}

std::string DataR3R2::ToString()
{
    std::stringstream ss;
    ss << "(" << position << ") " << value(0) << " " << value(1);
    return ss.str();
}
