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

#include "Helpers.h"
#include "LinearInterpolatedOrientation.h"
#include <stdexcept>
using namespace math;


#define M_PI_F (static_cast<float>(M_PI))

Eigen::Vector3f math::Helpers::GetOrthonormalVectors(
        Eigen::Vector3f vec, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2)
{
    vec = vec.normalized();
    dir1 = vec.cross(Eigen::Vector3f::UnitZ());
    if (dir1.norm() < 0.1f)
    {
        dir1 = vec.cross(Eigen::Vector3f::UnitY());
    }
    dir1 = -dir1.normalized();
    dir2 = vec.cross(dir1).normalized();
    return vec;
}

float Helpers::ShiftAngle0_2PI(float a)
{
    while (a < 0) a += 2*M_PI;
    while (a > 2 * M_PI_F)
        a -= 2*M_PI;
    return a;
}

float Helpers::AngleModPI(float value)
{
    return ShiftAngle0_2PI(value + M_PI_F) - M_PI_F;
}

void Helpers::GetIndex(float t, float minT, float maxT, int count, int &i, float &f)
{
    if (minT == maxT)
    {
        f = i = 0;
        return;
    }
    f = ((t - minT) / (maxT - minT)) * (count - 1);
    i = std::max(0, std::min(count - 2, static_cast<int>(f)));
    f -= i;
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

Eigen::Vector3f Helpers::Lerp(const Eigen::Vector3f &a, const Eigen::Vector3f &b, float f)
{
    return Eigen::Vector3f(Lerp(a(0), b(0), f),
                           Lerp(a(1), b(1), f),
                           Lerp(a(2), b(2), f));
}

Eigen::Quaternionf Helpers::Lerp(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b, float f)
{
    return LinearInterpolatedOrientation(a, b, 0, 1, false).Get(f);
}

Eigen::Quaternionf Helpers::LerpClamp(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b, float f)
{
    return LinearInterpolatedOrientation(a, b, 0, 1, true).Get(f);
}

float Helpers::ILerp(float a, float b, float f)
{
    return (f - a) / (b - a);
}

float Helpers::Lerp(float a, float b, int min, int max, int val)
{
    if (min == max) return a; //TODO
    return Lerp(a, b, (val - min) / static_cast<float>(max - min));
}

float Helpers::Angle(Eigen::Vector2f v) 
{ 
    return static_cast<float>(std::atan2(v.y(), v.x())); 
}

int Helpers::Sign(float x)
{
    return x > 0 ? 1 : (x == 0 ? 0 : -1);
}

void Helpers::AssertNormalized(Eigen::Vector3f vec, float epsilon)
{
    float len = vec.squaredNorm();
    if (len < 1 - epsilon || len > 1 + epsilon) throw std::runtime_error("Vector is not normalized");
}

std::vector<float> Helpers::FloatRange(float start, float end, int steps)
{
    std::vector<float> result;
    for (int i = 0; i < steps+1; ++i) {
        result.push_back(Lerp(start, end, i/static_cast<float>(steps)));
    }
    return result;
}

std::vector<Eigen::Vector3f> Helpers::VectorRangeSymmetric(float start, float end, int steps)
{
    std::vector<float> vals = FloatRange(start, end, steps);
    return VectorRange(vals, vals, vals);
}

std::vector<Eigen::Vector3f> Helpers::VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals)
{
    std::vector<Eigen::Vector3f> result;
    for (float x : xvals)
    {
        for (float y : yvals)
        {
            for (float z : zvals)
            {
                result.push_back(Eigen::Vector3f(x, y, z));
            }
        }
    }
    return result;
}

float Helpers::SmallestAngle(Eigen::Vector3f a, Eigen::Vector3f b)
{
    return static_cast<float>(std::atan2(a.cross(b).norm(), a.dot(b)));
}

Eigen::Vector3f Helpers::CwiseMin(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return a.cwiseMin(b);
}

Eigen::Vector3f Helpers::CwiseMax(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return a.cwiseMax(b);
}

Eigen::Vector3f Helpers::CwiseDivide(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return a.cwiseQuotient(b);
}

Eigen::Vector3f Helpers::Average(const std::vector<Eigen::Vector3f>& vectors)
{
    Eigen::Vector3f avg = Eigen::Vector3f::Zero();
    if (vectors.size() == 0) return avg;
    for (const Eigen::Vector3f& v : vectors)
    {
        avg += v;
    }
    avg /= vectors.size();
    return avg;
}

void Helpers::Swap(float& a,float& b)
{
    std::swap(a, b);
}

Eigen::Matrix4f Helpers::CreatePose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& ori)
{
    return toPose(pos, ori);
}

Eigen::Matrix4f Helpers::CreatePose(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori)
{
    return toPose(pos, ori);
}


Eigen::Matrix3f Helpers::GetRotationMatrix(const Eigen::Vector3f& source, const Eigen::Vector3f& target)
{
    return Eigen::Quaternionf::FromTwoVectors(source, target).toRotationMatrix();
}

Eigen::Matrix3f Helpers::RotateOrientationToFitVector(
        const Eigen::Matrix3f& ori, const Eigen::Vector3f& localSource, 
        const Eigen::Vector3f& globalTarget)
{
    Eigen::Vector3f vec = ori * localSource;
    return GetRotationMatrix(vec, globalTarget) * ori;
}

Eigen::Vector3f Helpers::CreateVectorFromCylinderCoords(float r, float angle, float z)
{
    return Eigen::Vector3f(r * std::cos(angle), r * std::sin(angle), z);
}

Eigen::Matrix4f math::Helpers::TranslatePose(const Eigen::Matrix4f& pose, const Eigen::Vector3f& offset)
{
    Eigen::Matrix4f p = pose;
    posBlock(p) += offset;
    return p;
}

Eigen::Vector3f Helpers::TransformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f& pos)
{
    return (transform * pos.homogeneous()).head<3>();
}

Eigen::Vector3f Helpers::TransformDirection(const Eigen::Matrix4f& transform, const Eigen::Vector3f& dir)
{
    return oriBlock(transform) * dir;
}

Eigen::Matrix3f Helpers::TransformOrientation(const Eigen::Matrix4f& transform, const Eigen::Matrix3f& ori)
{
    return oriBlock(transform) * ori;
}

float Helpers::Distance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float rad2mmFactor)
{
    Eigen::AngleAxisf aa(oriBlock(b) * oriBlock(a).inverse());
    float dist = (posBlock(a) - posBlock(b)).norm();
    return dist + AngleModPI(aa.angle()) * rad2mmFactor;
}

Eigen::Vector3f Helpers::GetPosition(const Eigen::Matrix4f& pose)
{
    return posBlock(pose);
}

Eigen::Matrix3f Helpers::GetOrientation(const Eigen::Matrix4f& pose)
{
    return oriBlock(pose);
}

void Helpers::InvertPose(Eigen::Matrix4f& pose)
{
    oriBlock(pose).transposeInPlace();
    posBlock(pose) = - oriBlock(pose) * posBlock(pose);
}

Eigen::VectorXf Helpers::LimitVectorLength(const Eigen::VectorXf& vec, const Eigen::VectorXf& maxLen)
{
    if(maxLen.rows() != 1 && maxLen.rows() != vec.rows())
    {
        throw std::invalid_argument("maxLen.rows != 1 and != maxLen.rows");
    }
    float scale = 1;
    for(int i = 0; i < vec.rows(); i++)
    {
        int j = maxLen.rows() == 1 ? 0 : i;
        if(std::abs(vec(i)) > maxLen(j) && maxLen(j) >= 0)
        {
            scale = std::min(scale, maxLen(j) / std::abs(vec(i)));
        }
    }
    return vec / scale;
}

float Helpers::rad2deg(float rad)
{
    return rad * 180.0f / M_PI_F;
}

float Helpers::deg2rad(float deg)
{
    return deg * M_PI_F / 180.0f;
}
