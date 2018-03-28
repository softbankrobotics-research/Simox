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

    virtual Eigen::Vector3f GetDdu(float u, float v)= 0;
    virtual Eigen::Vector3f GetDdv(float u, float v)= 0;

    Eigen::Vector3f GetNormal(float u, float v);

    Plane GetContactPlane(float u, float v);

    Contact GetContact(float u, float v);
    virtual void GetUV(Eigen::Vector3f pos, float &u, float &v)= 0;

    Eigen::Vector3f GetNormal(Eigen::Vector2f uv);
    Eigen::Vector3f GetDdu(Eigen::Vector2f uv);
    Eigen::Vector3f GetDdv(Eigen::Vector2f uv);
    Plane GetContactPlane(Eigen::Vector2f uv) ;
    Contact GetContact(Eigen::Vector2f uv) ;
    Eigen::Vector2f GetUVFromPos(Eigen::Vector3f pos) ;
    Eigen::Vector3f GetVector(Eigen::Vector2f pos, Eigen::Vector2f dir);

    float GetSquareDistance(Eigen::Vector3f pos, ProjectionType projection);

    float GetDistance(Eigen::Vector3f pos, ProjectionType projection);

    Eigen::Vector3f GetPointOnFunction(Eigen::Vector3f pos, ProjectionType projection);
    Eigen::Vector3f ProjectPointOntoFunction(Eigen::Vector3f pos);
    Eigen::Vector3f FindClosestPoint(Eigen::Vector3f pos, float epsilon = 0.001f);
    LineR2 ProjectVectorToUV(Eigen::Vector3f pos, Eigen::Vector3f dir);


private:
   // void Step(Eigen::Vector3f pos, float u0, float v0, float& u1,  float& v1);
};
}

#endif // math_HapticExplorationLibrary_AbstractFunctionR2R3
