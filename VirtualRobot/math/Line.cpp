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

#include "Line.h"
#include "Triangle.h"
#include "Primitive.h"
#include "float.h"
#include "Helpers.h"

using namespace math;



Line::Line(Eigen::Vector3f pos, Eigen::Vector3f dir)
    : pos(pos), dir(dir)
{
}

Line Line::Normalized() const
{
    return Line(pos, dir.normalized());
}

Eigen::Vector3f Line::Get(float t)
{
    return pos + t * dir;
}

Eigen::Vector3f Line::GetDerivative(float)
{
    return dir;
}

Eigen::Vector3f Line::GetClosestPoint(Eigen::Vector3f p) const
{
    return pos - (pos - p).dot(dir) * dir / dir.squaredNorm();
}

float Line::GetT(Eigen::Vector3f p) const
{
    return (p - pos).dot(dir) / dir.squaredNorm();

}

std::string Line::ToString() const
{
    std::stringstream ss;
    ss << "(" << pos << ") (" << dir << ")";
    return ss.str();
}


//https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool Line::IntersectsTriangle(Triangle tri, float &out) const
{
    const float EPS = 0.000; //TODO
    Eigen::Vector3f e1, e2;  //Edge1, Edge2
    Eigen::Vector3f P, Q, T;
    float det, inv_det, u, v;
    float t;
    Eigen::Vector3f V1 = tri.P1();
    Eigen::Vector3f V2 = tri.P2();
    Eigen::Vector3f V3 = tri.P3();

    //Find vectors for two edges sharing V1
    e1 = V2 -V1;
    e2 = V3 -V1;

    //Begin calculating determinant - also used to calculate u parameter
    P = dir.cross(e2);
    //if determinant is near zero, ray lies in plane of triangle
    det =e1.dot( P);
    //NOT CULLING
    if(det > -EPS && det < EPS) return 0;
    inv_det = 1.f / det;

    //calculate distance from V1 to ray origin
    T = pos - V1;

    //Calculate u parameter and test bound
    u = T.dot(P) * inv_det;
    //The intersection lies outside of the triangle
    if(u < 0.f || u > 1.f) return 0;

    //Prepare to test v parameter
   Q = T.cross(e1);

    //Calculate V parameter and test bound
    v = dir.dot(Q) * inv_det;
    //The intersection lies outside of the triangle
    if(v < 0.f || u + v  > 1.f) return 0;

    t = e2.dot(Q) * inv_det;

    if(t > EPS) { //ray intersection
      out = t;
      return 1;
    }

    // No hit, no win
    return 0;
}

bool Line::IntersectsPrimitive(PrimitivePtr p, float &out) const
{
    float min = FLT_MAX;
    float t;
    for(Triangle tri : *p){
        if (IntersectsTriangle(tri, t)){
            if(t<min) min = t;
        }
    }
    out = min;
    return out < FLT_MAX;
}

Line Line::FromPoints(Eigen::Vector3f p1, Eigen::Vector3f p2)
{
    return Line(p1, p2 - p1);
}

Line Line::FromPoses(const Eigen::Matrix4f &p1, const Eigen::Matrix4f &p2)
{
    return FromPoints(p1.block<3,1>(0,3), p2.block<3,1>(0,3));
}

Line Line::Transform(const Eigen::Matrix4f &pose) const
{
    return Line(Helpers::TransformPosition(pose, pos), Helpers::TransformDirection(pose, dir));
}






