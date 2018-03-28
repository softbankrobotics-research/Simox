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

#ifndef math_HapticExplorationLibrary_AbstractFunctionR2R3
#define math_HapticExplorationLibrary_AbstractFunctionR2R3

#include "SimpleAbstractFunctionR2R3.h"



namespace math
{


class AbstractFunctionR2R3:
        public SimpleAbstractFunctionR2R3
{
public:
    enum ProjectionType { SimpleProjection, FindClosestPointType };

    AbstractFunctionR2R3();

    virtual Vec3 GetDdu(float u, float v)= 0;
    virtual Vec3 GetDdv(float u, float v)= 0;

    Vec3 GetNormal(float u, float v);

    Plane GetContactPlane(float u, float v);

    Contact GetContact(float u, float v);
    virtual void GetUV(Vec3 pos, float &u, float &v)= 0;

    Vec3 GetNormal(Vec2 uv);
    Vec3 GetDdu(Vec2 uv);
    Vec3 GetDdv(Vec2 uv);
    Plane GetContactPlane(Vec2 uv) ;
    Contact GetContact(Vec2 uv) ;
    Vec2 GetUVFromPos(Vec3 pos) ;
    Vec3 GetVector(Vec2 pos, Vec2 dir);

    float GetSquareDistance(Vec3 pos, ProjectionType projection);

    float GetDistance(Vec3 pos, ProjectionType projection);

    Vec3 GetPointOnFunction(Vec3 pos, ProjectionType projection);
    Vec3 ProjectPointOntoFunction(Vec3 pos);
    Vec3 FindClosestPoint(Vec3 pos, float epsilon = 0.001f);
    LineR2 ProjectVectorToUV(Vec3 pos, Vec3 dir);


private:
   // void Step(Vec3 pos, float u0, float v0, float& u1,  float& v1);
};
}

#endif // math_HapticExplorationLibrary_AbstractFunctionR2R3
