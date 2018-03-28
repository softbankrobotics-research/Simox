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

#ifndef math_HapticExplorationLibrary_Plane
#define math_HapticExplorationLibrary_Plane

#include "MathForwardDefinitions.h"
#include "AbstractFunctionR2R3.h"
#include "Line.h"
#include "ImplicitPlane.h"





namespace math
{

class Plane :
        public AbstractFunctionR2R3
{

public:
    Plane(Vec3 pos, Vec3 dir1, Vec3 dir2);


    Vec3 Pos() {return pos;}
    Vec3 Dir1(){return dir1;}
    Vec3 Dir2(){return dir2;}


    Vec3 GetPoint(float u, float v) override;
    Vec3 GetDdu(float u, float v) override;
    Vec3 GetDdv(float u, float v) override;
    void GetUV(Vec3 pos, float &u, float &v) override;
    Vec3 GetNormal();
    Plane SwappedDirections();
    Plane Normalized();
    ImplicitPlane ToImplicit();
    Matrix3f GetRotationMatrix();
    std::string ToString();
    void GetUVW(Vec3 pos, float &u, float &v, float &w);
    float GetW(Vec3 pos);
    float Intersect(Line line, bool* exists);
    Line Intersect(Plane p2);

    static Plane FromNormal(Vec3 pos, Vec3 normal);
private:
    Vec3 pos;
    Vec3 dir1;
    Vec3 dir2;

};
}

#endif // math_HapticExplorationLibrary_Plane
