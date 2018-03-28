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

#ifndef math_HapticExplorationLibrary_LineR2
#define math_HapticExplorationLibrary_LineR2

#include "MathForwardDefinitions.h"





namespace math
{

class LineR2
{
public:
    LineR2(Vec3 pos, Vec3 dir);

    Vec3 Pos(){return pos;}
    Vec3 Dir(){return dir;}

    LineR2 Normalized ();
    Vec3 Get(float t);
    Vec3 GetDerivative(float t);
    Vec3 GetClosestPoint(Vec3 p);
    float GetT(Vec3 p);
    std::string ToString();
    //Vec3Opt Intersect(const LineR2& l2);
    //static bool Intersect(const Vec3& pos1, const Vec3& dir1, const Vec3& pos2, const Vec3& dir2, float& t,float& u);

    //static bool Intersect(LineR2 l1, LineR2 l2, out float t, out float u);


private:
    Vec3 pos;
    Vec3 dir;

};
}

#endif // math_HapticExplorationLibrary_LineR2
