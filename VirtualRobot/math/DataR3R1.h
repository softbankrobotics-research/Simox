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

#ifndef math_DataR3R1
#define math_DataR3R1

#include "MathForwardDefinitions.h"

namespace math
{

    class DataR3R1
    {
    public:
        Vec3 Position(){return position;}
        float Value(){return value;}
        DataR3R1(Vec3 position, float value);
        std::string ToString();
    private:
        Vec3 position;
        float value;
    };
}

#endif // math_DataR3R1
