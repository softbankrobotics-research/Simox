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

#pragma once

#include "MathForwardDefinitions.h"

namespace math
{
    class Helpers
    {
    public:


        static Eigen::Vector3f GetOrthonormalVectors(Eigen::Vector3f vec, Eigen::Vector3f& dir1, Eigen::Vector3f& dir2);
        static float ShiftAngle0_2PI(float a);
        static float AngleModPI(float value);
        static void GetIndex(float t, float minT, float maxT, int count, int& i, float& f);
        static float Clamp(float min, float max, float value);
        static float Lerp(float a, float b, float f);
        static Eigen::Vector3f Lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float f);
        static Eigen::Quaternionf Lerp(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b, float f);
        static float ILerp(float a, float b, float f);
        static float Lerp(float a, float b, int min, int max, int val);
        static float Angle(Eigen::Vector2f v);
        static int Sign(float x);
        static void AssertNormalized(Eigen::Vector3f vec, float epsilon = 0.05f);
        static std::vector<float> FloatRange(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRangeSymmetric(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals);
        static float SmallestAngle(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseMin(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseMax(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseDivide(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f Average(std::vector<Eigen::Vector3f> vectors);
        static void Swap(float &a,float &b);
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& ori);
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori);
        static Eigen::Matrix3f GetRotationMatrix(const Eigen::Vector3f &source, const Eigen::Vector3f &target);
        static Eigen::Matrix3f RotateOrientationToFitVector(const Eigen::Matrix3f& ori, const Eigen::Vector3f &localSource, const Eigen::Vector3f &globalTarget);
        static Eigen::Vector3f CreateVectorFromCylinderCoords(float r, float angle, float z);
        static Eigen::Matrix4f TranslatePose(Eigen::Matrix4f pose, const Eigen::Vector3f& offset);
        static Eigen::Vector3f TransformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f &pos);
        static Eigen::Vector3f TransformDirection(const Eigen::Matrix4f& transform, const Eigen::Vector3f &dir);
        static Eigen::Matrix3f TransformOrientation(const Eigen::Matrix4f& transform, const Eigen::Matrix3f &ori);
        static float Distance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float rad2mmFactor);
        static Eigen::Vector3f GetPosition(const Eigen::Matrix4f& pose);
        static Eigen::Matrix3f GetOrientation(const Eigen::Matrix4f& pose);

    private:
    };
}

