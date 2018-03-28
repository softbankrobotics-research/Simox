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

#ifndef math_HapticExplorationLibrary_AbstractFunctionR1R3
#define math_HapticExplorationLibrary_AbstractFunctionR1R3

#include "SimpleAbstractFunctionR1R3.h"



namespace math
{
    class AbstractFunctionR1R3 :
            public SimpleAbstractFunctionR1R3
    {
    public:
        AbstractFunctionR1R3();
        virtual Vec3 GetDerivative(float t)= 0;
        float FindClosestPoint(Vec3 p, float t1, float t2, int segments);
        float MoveLengthOnCurve(float x, float l, int steps);
        float GetLength(float t1, float t2, int steps);
        std::vector<Vec3> Sample(float t1, float t2, int segments);
        std::vector<float> Segments(float t1, float t2, int segments);
    private:
    };
}

#endif // math_HapticExplorationLibrary_AbstractFunctionR1R3
