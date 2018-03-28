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


using namespace math;

AbstractFunctionR2R3::AbstractFunctionR2R3()
{
}

Vec3 AbstractFunctionR2R3::GetNormal(float u, float v)
{
    Vec3 dduVec = GetDdu(u, v);
    Vec3 ddvVec = GetDdv(u, v);
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

Vec3 AbstractFunctionR2R3::GetNormal(Vec2 uv) { return GetNormal(uv.x(), uv.y()); }

Vec3 AbstractFunctionR2R3::GetDdu(Vec2 uv) { return GetDdu(uv.x(), uv.y()); }

Vec3 AbstractFunctionR2R3::GetDdv(Vec2 uv) { return GetDdv(uv.x(), uv.y()); }

Vec2 AbstractFunctionR2R3::GetUVFromPos(Vec3 pos)
{
    float u, v;
    GetUV(pos,  u,  v);
    return  Vec2(u, v);
}

Vec3 AbstractFunctionR2R3::GetVector(Vec2 pos, Vec2 dir) { return GetDdu(pos) * dir.x() + GetDdv(pos) * dir.y(); }
