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

