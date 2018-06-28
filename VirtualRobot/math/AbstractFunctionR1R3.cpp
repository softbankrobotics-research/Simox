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

#include "AbstractFunctionR1R3.h"

using namespace math;

AbstractFunctionR1R3::AbstractFunctionR1R3()
{
}


float AbstractFunctionR1R3::FindClosestPoint(Eigen::Vector3f p, float t1, float t2, int segments)
{
    float minT = t1;
    float minD = (Get(minT)-p).squaredNorm();
    for (int i = 1; i <= segments; i++)
    {
        float t = t1 + (t2 - t1) * i / segments;
        float d = (Get(t)-p).squaredNorm();
        if (d < minD)
        {
            minT = t;
            minD = d;
        }
    }
    return minT;

}


float AbstractFunctionR1R3::MoveLengthOnCurve(float x, float l, int steps)
{
    float dl = l / steps;
    for (int i = 0; i < steps; i++)
    {
        float d = GetDerivative(x).norm();
        x += dl / (float)std::sqrt(1 + d * d); // move by dl on curve: tangential vector: (1, df/dx), normalize to dl
    }
    return x;
}


float AbstractFunctionR1R3::GetLength(float t1, float t2, int steps)
{
    Eigen::Vector3f p0 = Get(t1);
    float l = 0;
    for (int i = 1; i <= steps; i++)
    {
        float t = t1 + (t2 - t1) / steps * i;
        Eigen::Vector3f p1 = Get(t);
        l += (p1 - p0).norm();
        p0 = p1;
    }
    return l;
}


std::vector<Eigen::Vector3f> AbstractFunctionR1R3::Sample(float t1, float t2, int segments)
{
    std::vector<Eigen::Vector3f> result;
    for(float f : Segments(t1, t2, segments)){
        result.push_back(Get(f));
    }
    return result;
}


std::vector<float> AbstractFunctionR1R3::Segments(float t1, float t2, int segments)
{
    std::vector<float> result;

    for (int i = 0; i <= segments; i++)
    {
        result.push_back(t1 + (t2 - t1) * i / segments);
    }
    return result;
}
