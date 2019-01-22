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
        static Eigen::Quaternionf LerpClamp(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b, float f);
        static float ILerp(float a, float b, float f);
        static float Lerp(float a, float b, int min, int max, int val);
        static float Angle(Eigen::Vector2f v);
        static int Sign(float x);
        static void AssertNormalized(Eigen::Vector3f vec, float epsilon = 0.05f);
        static std::vector<float> FloatRange(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRangeSymmetric(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals);
        static float SmallestAngle(Eigen::Vector3f a, Eigen::Vector3f b);
        
        static Eigen::Vector3f CwiseMin(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f CwiseMax(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f CwiseDivide(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
        static Eigen::Vector3f Average(const std::vector<Eigen::Vector3f>& vectors);
        static void Swap(float &a,float &b);
        
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& ori);
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori);
        static Eigen::Matrix3f GetRotationMatrix(const Eigen::Vector3f &source, const Eigen::Vector3f &target);
        static Eigen::Matrix3f RotateOrientationToFitVector(const Eigen::Matrix3f& ori, const Eigen::Vector3f &localSource, const Eigen::Vector3f &globalTarget);
        static Eigen::Vector3f CreateVectorFromCylinderCoords(float r, float angle, float z);
        
        static Eigen::Matrix4f TranslatePose(const Eigen::Matrix4f &pose, const Eigen::Vector3f& offset);
        static Eigen::Vector3f TransformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f &pos);
        static Eigen::Vector3f TransformDirection(const Eigen::Matrix4f& transform, const Eigen::Vector3f &dir);
        static Eigen::Matrix3f TransformOrientation(const Eigen::Matrix4f& transform, const Eigen::Matrix3f &ori);
        static float Distance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float rad2mmFactor);

        static Eigen::Vector3f GetPosition(const Eigen::Matrix4f& pose);
        static Eigen::Matrix3f GetOrientation(const Eigen::Matrix4f& pose);
        static Eigen::VectorXf LimitVectorLength(const Eigen::VectorXf& vec, const Eigen::VectorXf& maxLen);

        static float rad2deg(float rad);
        static float deg2rad(float deg);

        
        /// Get the position block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 1> posBlock(Eigen::MatrixBase<Derived>& pose);
        
        /// Get the position block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 1>
        posBlock(const Eigen::MatrixBase<Derived>& pose);
    
        
        /// Get the orientation block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 3> oriBlock(Eigen::MatrixBase<Derived>& pose);
        
        /// Get the orientation block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 3>
        oriBlock(const Eigen::MatrixBase<Derived>& pose);
    
        
        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename RotDerived>
        static Eigen::Matrix4f toPose(const Eigen::MatrixBase<PosDerived>& pos,
                                      const Eigen::MatrixBase<RotDerived>& ori);
        
        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename RotDerived>
        static Eigen::Matrix4f toPose(const Eigen::MatrixBase<PosDerived>& pos,
                                      const Eigen::RotationBase<RotDerived, 3>& ori);
        
    private:
        
    };

    
    
    template <typename Derived>
    Eigen::Block<Derived, 3, 1> Helpers::posBlock(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 1>(pose.derived(), 0, 3);
    }
    
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 1> Helpers::posBlock(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 1>(pose.derived(), 0, 3);
    }

    
    template <typename Derived>
    Eigen::Block<Derived, 3, 3> Helpers::oriBlock(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 3>(pose.derived(), 0, 0);
    }
    
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 3> Helpers::oriBlock(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 3>(pose.derived(), 0, 0);
    }

    
    template <typename PosDerived, typename RotDerived>
    Eigen::Matrix4f Helpers::toPose(const Eigen::MatrixBase<PosDerived>& pos,
                                    const Eigen::MatrixBase<RotDerived>& ori)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<PosDerived>, 3, 1);
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<RotDerived>, 3, 3);
        Eigen::Matrix4f pose = pose.Identity();
        posBlock(pose) = pos;
        oriBlock(pose) = ori;
        return pose;
    }
    
    template <typename PosDerived, typename RotDerived>
    Eigen::Matrix4f Helpers::toPose(const Eigen::MatrixBase<PosDerived>& pos,
                                    const Eigen::RotationBase<RotDerived, 3>& ori)
    {
        return toPose(pos, ori.toRotationMatrix());
    }
    
}

