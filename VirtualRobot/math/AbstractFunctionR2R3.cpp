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

#include "AbstractFunctionR2R3.h"
#include "Contact.h"
#include "Plane.h"
#include <stdexcept>

using namespace math;

AbstractFunctionR2R3::AbstractFunctionR2R3()
{
}

Eigen::Vector3f AbstractFunctionR2R3::GetNormal(float u, float v)
{
    Eigen::Vector3f dduVec = GetDdu(u, v);
    Eigen::Vector3f ddvVec = GetDdv(u, v);
    return dduVec.cross(ddvVec);
}

Plane AbstractFunctionR2R3::GetContactPlane(float u, float v)
{
    return Plane(GetPoint(u, v), GetDdu(u, v), GetDdv(u, v));
}

Contact AbstractFunctionR2R3::GetContact(float u, float v)
{
     return Contact(GetPoint(u, v), GetNormal(u, v));
}

Eigen::Vector3f AbstractFunctionR2R3::GetNormal(Eigen::Vector2f uv) { return GetNormal(uv.x(), uv.y()); }

Eigen::Vector3f AbstractFunctionR2R3::GetDdu(Eigen::Vector2f uv) { return GetDdu(uv.x(), uv.y()); }

Eigen::Vector3f AbstractFunctionR2R3::GetDdv(Eigen::Vector2f uv) { return GetDdv(uv.x(), uv.y()); }

Eigen::Vector2f AbstractFunctionR2R3::GetUVFromPos(Eigen::Vector3f pos)
{
    float u, v;
    GetUV(pos,  u,  v);
    return  Eigen::Vector2f(u, v);
}

Eigen::Vector3f AbstractFunctionR2R3::GetVector(Eigen::Vector2f pos, Eigen::Vector2f dir) { return GetDdu(pos) * dir.x() + GetDdv(pos) * dir.y(); }

float AbstractFunctionR2R3::GetDistance(Eigen::Vector3f pos, AbstractFunctionR2R3::ProjectionType projection)
{
    return (GetPointOnFunction(pos, projection) - pos).norm();
}

Eigen::Vector3f AbstractFunctionR2R3::GetPointOnFunction(Eigen::Vector3f pos, AbstractFunctionR2R3::ProjectionType projection)
{
    switch (projection)
    {
        case SimpleProjection:
            return ProjectPointOntoFunction(pos);
        case FindClosestPointType:
            return FindClosestPoint(pos);
        default:
            throw std::runtime_error("invalid case");
    }
}

Eigen::Vector3f AbstractFunctionR2R3::ProjectPointOntoFunction(Eigen::Vector3f pos)
{
    float u, v;
    GetUV(pos, u, v);
    return GetPoint(u, v);
}

Eigen::Vector3f AbstractFunctionR2R3::FindClosestPoint(Eigen::Vector3f pos, float epsilon)
{
    throw std::runtime_error("FindClosestPoint is not implemented");
}
