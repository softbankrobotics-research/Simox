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


Eigen::Vector3f math::Helpers::GetOrthonormalVectors(Eigen::Vector3f vec, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2)
{
    vec = vec.normalized();
    dir1 = vec.cross(Eigen::Vector3f(0,0,1));
    if (dir1.norm() < 0.1f)
    {
        dir1 = vec.cross(Eigen::Vector3f(0,1,0));
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

float Helpers::AngleModPI(float value)
{
    return ShiftAngle0_2PI(value + M_PI) - M_PI;
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
    return Lerp(a, b, (val - min) / (float)(max - min));
}

float Helpers::Angle(Eigen::Vector2f v) { return (float)std::atan2(v.y(), v.x());}

int Helpers::Sign(float x){
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
        result.push_back(Lerp(start, end, i/(float) steps));
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
    return (float)std::atan2(a.cross(b).norm(), a.dot(b));
}

Eigen::Vector3f Helpers::CwiseMin(Eigen::Vector3f a, Eigen::Vector3f b)
{
    return Eigen::Vector3f(std::min(a.x(), b.x()),
                std::min(a.y(), b.y()),
                std::min(a.z(), b.z()));
}

Eigen::Vector3f Helpers::CwiseMax(Eigen::Vector3f a, Eigen::Vector3f b)
{
    return Eigen::Vector3f(std::max(a.x(), b.x()),
                std::max(a.y(), b.y()),
                std::max(a.z(), b.z()));

}

Eigen::Vector3f Helpers::CwiseDivide(Eigen::Vector3f a, Eigen::Vector3f b)
{
    return Eigen::Vector3f(a.x()/ b.x(),
                a.y()/ b.y(),
                a.z()/ b.z());
}

Eigen::Vector3f Helpers::Average(std::vector<Eigen::Vector3f> vectors)
{
    Eigen::Vector3f avg = Eigen::Vector3f(0,0,0);
    if(vectors.size() == 0) return avg;
    for(Eigen::Vector3f v : vectors){
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

Eigen::Matrix4f Helpers::CreatePose(const Eigen::Vector3f &pos, const Eigen::Quaternionf &ori)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = ori.toRotationMatrix();
    pose.block<3,1>(0,3) = pos;
    return pose;
}

Eigen::Matrix4f Helpers::CreatePose(const Eigen::Vector3f &pos, const Eigen::Matrix3f &ori)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = ori;
    pose.block<3,1>(0,3) = pos;
    return pose;
}


Eigen::Matrix3f Helpers::GetRotationMatrix(const Eigen::Vector3f & source, const Eigen::Vector3f & target)
{
    Eigen::Vector3f src = source.normalized();
    Eigen::Vector3f tgt = target.normalized();
    float angle = acos(src.dot(tgt));
    if(fabs(angle) < 0.001f)
    {
        return Eigen::Matrix3f::Identity();
    }
    Eigen::Vector3f axis = src.cross(tgt);
    axis.normalize();
    return Eigen::AngleAxisf(angle, axis).toRotationMatrix();
}

Eigen::Matrix3f Helpers::RotateOrientationToFitVector(const Eigen::Matrix3f &ori, const Eigen::Vector3f &localSource, const Eigen::Vector3f &globalTarget)
{
    Eigen::Vector3f vec = ori * localSource;
    return GetRotationMatrix(vec, globalTarget) * ori;
}

Eigen::Vector3f Helpers::CreateVectorFromCylinderCoords(float r, float angle, float z)
{
    return Eigen::Vector3f(r * cos(angle), r * sin(angle), z);
}

Eigen::Matrix4f math::Helpers::TranslatePose(const Eigen::Matrix4f &pose, const Eigen::Vector3f &offset)
{
    Eigen::Matrix4f p = pose;
    p.block<3, 1>(0, 3) += offset;
    return p;
}

Eigen::Vector3f Helpers::TransformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f& pos)
{
    return (transform * Eigen::Vector4f(pos(0), pos(1), pos(2), 1)).block<3,1>(0,0);
}

Eigen::Vector3f Helpers::TransformDirection(const Eigen::Matrix4f& transform, const Eigen::Vector3f& dir)
{
    return transform.block<3,3>(0,0) * dir;
}

Eigen::Matrix3f Helpers::TransformOrientation(const Eigen::Matrix4f& transform, const Eigen::Matrix3f& ori)
{
    return transform.block<3,3>(0,0) * ori;
}

float Helpers::Distance(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b, float rad2mmFactor)
{
    Eigen::AngleAxisf aa(b.block<3,3>(0,0) * a.block<3,3>(0,0).inverse());
    float dist = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
    return dist + AngleModPI(aa.angle()) * rad2mmFactor;
}

Eigen::Vector3f Helpers::GetPosition(const Eigen::Matrix4f& pose)
{
    return pose.block<3, 1>(0, 3);
}

Eigen::Matrix3f Helpers::GetOrientation(const Eigen::Matrix4f& pose)
{
    return pose.block<3, 3>(0, 0);
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
    return rad * (float)(180.0 / M_PI);
}

float Helpers::deg2rad(float deg)
{
    return deg * (float)(M_PI / 180.0);
}
