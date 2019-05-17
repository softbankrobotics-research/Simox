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
        static void Swap(float& a,float& b);


        // POSE UTILITY
        
        /// Get the position block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 1> 
        Position(Eigen::MatrixBase<Derived>& pose);
        
        /// Get the position block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 1> 
        Position(const Eigen::MatrixBase<Derived>& pose);
    
        
        /// Get the orientation block from the given pose.
        template <typename Derived>
        static Eigen::Block<Derived, 3, 3> 
        Orientation(Eigen::MatrixBase<Derived>& pose);
        
        /// Get the orientation block from the given pose.
        template <typename Derived>
        static const Eigen::Block<const Derived, 3, 3> 
        Orientation(const Eigen::MatrixBase<Derived>& pose);
    
        
        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename OriDerived>
        static Eigen::Matrix4f 
        Pose(const Eigen::MatrixBase<PosDerived>& pos, const Eigen::MatrixBase<OriDerived>& ori);
        
        /// Build a pose matrix from the given position and orientation.
        template <typename PosDerived, typename OriDerived>
        static Eigen::Matrix4f 
        Pose(const Eigen::MatrixBase<PosDerived>& pos, const Eigen::RotationBase<OriDerived, 3>& ori);
        
        /// Build a pose matrix from the given position and identity orientation.
        template <typename PosDerived>
        static Eigen::Matrix4f 
        Pose(const Eigen::MatrixBase<PosDerived>& pos);
        
        
        /// Legacy shortcut for Pose().
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& ori);
        /// Legacy shortcut for Pose().
        static Eigen::Matrix4f CreatePose(const Eigen::Vector3f& pos, const Eigen::Matrix3f& ori);

        /// Legacy shortcut for Position() as getter.
        static Eigen::Vector3f GetPosition(const Eigen::Matrix4f& pose);
        /// Legacy shortcut for Orientation() as getter.
        static Eigen::Matrix3f GetOrientation(const Eigen::Matrix4f& pose);
        
        /// Translate the given pose by the given offset.
        static Eigen::Matrix4f TranslatePose(const Eigen::Matrix4f& pose, const Eigen::Vector3f& offset);
        
        /// Invert the given pose in-place.
        static void InvertPose(Eigen::Matrix4f& pose);
        /// Return the inverted of the given pose.
        template <typename Derived>
        static Eigen::Matrix4f InvertedPose(const Eigen::MatrixBase<Derived>& pose);
        
        /// Get a cartesian vector from cylinder coordinates.
        static Eigen::Vector3f CreateVectorFromCylinderCoords(float r, float angle, float z);
        /// Get a cartesian vector from cylinder coordinates.
        static Eigen::Vector3f CartesianFromCylinder(float radius, float angle, float height);
        /// Get a cartesian vector from sphere coordinates.
        static Eigen::Vector3f CartesianFromSphere(float radius, float elevation, float azimuth);
        
        /// Get a rotation matrix rotating source to target.
        static Eigen::Matrix3f GetRotationMatrix(const Eigen::Vector3f& source, const Eigen::Vector3f& target);
        
        static Eigen::Matrix3f RotateOrientationToFitVector(
                const Eigen::Matrix3f& ori, const Eigen::Vector3f& localSource, const Eigen::Vector3f& globalTarget);
        
        
        /// Transform the position by the transform.
        static Eigen::Vector3f TransformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f& pos);
        /// Transform the direction by the transform.
        static Eigen::Vector3f TransformDirection(const Eigen::Matrix4f& transform, const Eigen::Vector3f& dir);
        /// Transform the orientation by the transform.
        static Eigen::Matrix3f TransformOrientation(const Eigen::Matrix4f& transform, const Eigen::Matrix3f& ori);
        
        /// Indicates whether the matrix is orthogonal, i.e. matrix * matrix.transpose = identity.
        template <typename Derived>
        static bool IsMatrixOrthogonal(const Eigen::MatrixBase<Derived>& matrix, float precision = 1e-6f);
        
        /// Compute the closest orthogonal matrix to the given matrix.
        /// (Note: All rotation matrices must be orthogonal.)
        static Eigen::Matrix3f Orthogonalize(const Eigen::Matrix3f& matrix);
        
        /// Orthogonolize the given matrix using Householder QR decomposition.
        static Eigen::Matrix3f OrthogonalizeQR(const Eigen::Matrix3f& matrix);
        
        /// Orthogonolize the given matrix using Jacobi SVD decomposition.
        static Eigen::Matrix3f OrthogonalizeSVD(const Eigen::Matrix3f& matrix);
        
        /// Orthogonolize the orientation of the given pose, and sanitize its lower row.
        static Eigen::Matrix4f Orthogonalize(const Eigen::Matrix4f& pose);
        
        
        static float Distance(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b, float rad2mmFactor);

        static Eigen::VectorXf LimitVectorLength(const Eigen::VectorXf& vec, const Eigen::VectorXf& maxLen);


        // Rotation vectors:

        static Eigen::AngleAxisf GetAngleAxisFromTo(const Eigen::Matrix3f& start, const Eigen::Matrix3f& target);
        static Eigen::Vector3f GetRotationVector(const Eigen::Matrix3f& start, const Eigen::Matrix3f& target);
        static Eigen::Matrix3f RotationVectorToOrientation(const Eigen::Vector3f& rotation);

        
        
        /// Convert a value from radian to degree.
        static float rad2deg(float rad);
        /// Convert a value from degree to radian.
        static float deg2rad(float deg);
        
        /// Convert a value from radian to degree.
        template <typename ValueT>
        static ValueT rad2deg(const ValueT& rad);
        
        /// Convert a value from degree to radian.
        template <typename ValueT>
        static ValueT deg2rad(const ValueT& deg);

        
    private:
        
    };

    
    template <typename Derived>
    Eigen::Block<Derived, 3, 1> Helpers::Position(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 1>(pose.derived(), 0, 3);
    }
    
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 1> Helpers::Position(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 1>(pose.derived(), 0, 3);
    }

    
    template <typename Derived>
    Eigen::Block<Derived, 3, 3> Helpers::Orientation(Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<Derived, 3, 3>(pose.derived(), 0, 0);
    }
    
    template <typename Derived>
    const Eigen::Block<const Derived, 3, 3> Helpers::Orientation(const Eigen::MatrixBase<Derived>& pose)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 4);
        return Eigen::Block<const Derived, 3, 3>(pose.derived(), 0, 0);
    }

    
    template <typename PosDerived, typename OriDerived>
    Eigen::Matrix4f Helpers::Pose(const Eigen::MatrixBase<PosDerived>& pos,
                                  const Eigen::MatrixBase<OriDerived>& ori)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<PosDerived>, 3, 1);
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<OriDerived>, 3, 3);
        Eigen::Matrix4f pose = pose.Identity();
        Position(pose) = pos;
        Orientation(pose) = ori;
        return pose;
    }
    
    template <typename PosDerived, typename OriDerived>
    Eigen::Matrix4f Helpers::Pose(const Eigen::MatrixBase<PosDerived>& pos,
                                  const Eigen::RotationBase<OriDerived, 3>& ori)
    {
        return Pose(pos, ori.toRotationMatrix());
    }
    
    template <typename PosDerived>
    Eigen::Matrix4f Helpers::Pose(const Eigen::MatrixBase<PosDerived>& pos)
    {
        return Pose(pos, Eigen::Matrix3f::Identity());
    }
    
    
    template <typename Derived>
    Eigen::Matrix4f Helpers::InvertedPose(const Eigen::MatrixBase<Derived>& pose)
    {
        Eigen::Matrix4f inv = pose;
        InvertPose(inv);
        return inv;
    }
    
    template<typename Derived>
    bool Helpers::IsMatrixOrthogonal(const Eigen::MatrixBase<Derived>& matrix, float precision)
    {
        return (matrix * matrix.transpose()).isIdentity(precision);
    }
    
    
    template<typename ValueT>
    ValueT Helpers::rad2deg(const ValueT& rad)
    {
        return rad * (180.0 / M_PI);
    }
    
    template<typename ValueT>
    ValueT Helpers::deg2rad(const ValueT& deg)
    {
        return deg * (M_PI / 180.0);
    }
}


namespace Eigen
{
    template <typename Derived>
    std::ostream& operator<< (std::ostream& os, const QuaternionBase<Derived>& quat)
    {
        os << "[ " << quat.w() << " | "
           << quat.x() << " " << quat.y() << "  " << quat.z() << " ]";
        return os;
    }
}

