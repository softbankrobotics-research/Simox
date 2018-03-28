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
         Eigen::Vector3f P1() { return p1;  }
         Eigen::Vector3f P2() { return p2;  }
         int StepsX() { return stepsX;  }
         int StepsY() { return stepsY;  }
         int StepsZ() { return stepsZ;  }

        Grid3D(Eigen::Vector3f p1, Eigen::Vector3f p2, int stepsX, int stepsY, int stepsZ);
        static Grid3DPtr CreateFromBox(Eigen::Vector3f p1, Eigen::Vector3f p2, float stepLength);

        Eigen::Vector3f Get(int x, int y, int z);
        std::vector<Eigen::Vector3f> AllGridPoints();

    private :
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        int stepsX;
        int stepsY;
        int stepsZ;

    };
}

#endif // math_Grid3D
