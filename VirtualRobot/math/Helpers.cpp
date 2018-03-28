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

#include "Helpers.h"
#include <stdexcept>
using namespace math;


Vec3 math::Helpers::GetOrthonormalVectors(Vec3 vec, Vec3 &dir1, Vec3 &dir2)
{
    vec = vec.normalized();
    dir1 = vec.cross(Vec3(0,0,1));
    if (dir1.norm() < 0.1f)
    {
        dir1 = vec.cross(Vec3(0,1,0));
    }
    dir1 = -dir1.normalized();
    dir2 = vec.cross(dir1).normalized();
    return vec;
}

float Helpers::ShiftAngle0_2PI(float a)
{
    while (a < 0) a += 2*M_PI;
    while (a > 2*M_PI) a -= 2*M_PI;
    return a;
}

void Helpers::GetIndex(float t, float minT, float maxT, int count, int &i, float &f)
{
    if (minT == maxT)
    {
        f = i = 0;
        return;
    }
    f = ((t - minT) / (maxT - minT)) * (count - 1);
    i = std::max(0, std::min(count - 2, (int)f));
    f -= i;
}

float Helpers::FastDistribution(float x, float sigma2)
{
    return std::exp(-x*x /2 /sigma2);
}

float Helpers::Clamp(float min, float max, float value)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float Helpers::Lerp(float a, float b, float f)
{
    return a * (1 - f) + b * f;
}

Vec3 Helpers::Lerp(Vec3 a, Vec3 b, float f)
{
    return Vec3(Lerp(a(0), b(0), f),
                Lerp(a(1), b(1), f),
                Lerp(a(2), b(2), f));
}

float Helpers::Lerp(float a, float b, int min, int max, int val)
{
    if (min == max) return a; //TODO
    return Lerp(a, b, (val - min) / (float)(max - min));
}

float Helpers::Angle(Vec2 v) { return (float)std::atan2(v.y(), v.x());}

int Helpers::Sign(float x){
    return x > 0 ? 1 : (x == 0 ? 0 : -1);
}

void Helpers::AssertNormalized(Vec3 vec, float epsilon)
{
    float len = vec.squaredNorm();
    if (len < 1 - epsilon || len > 1 + epsilon) throw std::runtime_error("Vector is not normalized");
}

std::vector<float> Helpers::FloatRange(float start, float end, int steps)
{
    std::vector<float> result;
    for (int i = 0; i < steps+1; ++i) {
        result.push_back(Lerp(start, end, i/(float) steps));
    }
    return result;
}

std::vector<Vec3> Helpers::VectorRangeSymmetric(float start, float end, int steps)
{
    std::vector<float> vals = FloatRange(start, end, steps);
    return VectorRange(vals, vals, vals);
}

std::vector<Vec3> Helpers::VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals)
{
    std::vector<Vec3> result;
    for (float x : xvals)
    {
        for (float y : yvals)
        {
            for (float z : zvals)
            {
                result.push_back(Vec3(x, y, z));
            }
        }
    }
    return result;
}

float Helpers::SmallestAngle(Vec3 a, Vec3 b)
{
    return (float)std::atan2(a.cross(b).norm(), a.dot(b));
}

Vec3 Helpers::CwiseMin(Vec3 a, Vec3 b)
{
    return Vec3(std::min(a.x(), b.x()),
                std::min(a.y(), b.y()),
                std::min(a.z(), b.z()));
}

Vec3 Helpers::CwiseMax(Vec3 a, Vec3 b)
{
    return Vec3(std::max(a.x(), b.x()),
                std::max(a.y(), b.y()),
                std::max(a.z(), b.z()));

}

Vec3 Helpers::CwiseDivide(Vec3 a, Vec3 b)
{
    return Vec3(a.x()/ b.x(),
                a.y()/ b.y(),
                a.z()/ b.z());
}

Vec3 Helpers::Average(std::vector<Vec3> vectors)
{
    Vec3 avg = Vec3(0,0,0);
    if(vectors.size() == 0) return avg;
    for(Vec3 v : vectors){
        avg += v;
    }
    avg /= vectors.size();
    return avg;
}
void Helpers::Swap(float &a,float &b)
{
    float t = a;
    a = b;
    b = t;
}
