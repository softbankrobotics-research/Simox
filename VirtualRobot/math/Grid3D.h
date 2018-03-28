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

#ifndef math_Grid3D
#define math_Grid3D

#include "MathForwardDefinitions.h"

namespace math
{
    class Grid3D
    {
    public:
         Vec3 P1() { return p1;  }
         Vec3 P2() { return p2;  }
         int StepsX() { return stepsX;  }
         int StepsY() { return stepsY;  }
         int StepsZ() { return stepsZ;  }

        Grid3D(Vec3 p1, Vec3 p2, int stepsX, int stepsY, int stepsZ);
        static Grid3DPtr CreateFromBox(Vec3 p1, Vec3 p2, float stepLength);

        Vec3 Get(int x, int y, int z);
        std::vector<Vec3> AllGridPoints();

    private :
        Vec3 p1;
        Vec3 p2;
        int stepsX;
        int stepsY;
        int stepsZ;

    };
}

#endif // math_Grid3D
