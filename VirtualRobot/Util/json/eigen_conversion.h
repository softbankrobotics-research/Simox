#pragma once

#include <type_traits>
#include <iostream>
#include <typeinfo>

#include <Eigen/Core>

#include <VirtualRobot/math/Helpers.h>

#include "json.hpp"

/**
 * Provide to_json() and from_json() overloads for nlohmann::json,
 * which allows simple syntax like:
 * 
 * @code
 * Eigen::Matrix3f in, out;
 * 
 * nlohmann::json j;
 * j = in;
 * out = j.get<Eigen::Matrix3f>();
 * @endcode
 * 
 * @test VirtualRobotJsonEigenConversionTest
 * 
 * @see https://github.com/nlohmann/json#arbitrary-types-conversions
 */
namespace Eigen
{
    

    // MatrixBase (non-specialized)
    /// Writes the matrix as list of rows.
    template <typename Derived>
    void to_json(nlohmann::json& j, const MatrixBase<Derived>& matrix);
    
    /// Reads the matrix from list of rows.
    template <typename Derived>
    void from_json(const nlohmann::json& j, MatrixBase<Derived>& matrix);
    

    // Specialization for Vector3f (implemented in .cpp)
    
    /// Writes vector with x, y, z keys.
    template <>
    void to_json<Vector3f>(nlohmann::json& j, const MatrixBase<Vector3f>& vector);
    
    /// Reads vector from x, y, z keys.
    template <>
    void from_json<Vector3f>(const nlohmann::json& j, MatrixBase<Vector3f>& vector);
    
    
    // Specialization for Matrix4f as Transformation matrix (implemented in .cpp)
    
    /**
     * @brief Write a 4x4 matrix, with optimization for transformation matrices.
     * 
     * If the matrix is a transformation matrix, writes it as position
     * (Vector3f) and orientation (Quaternion).
     * Otherwise, uses the non-specialized method (list of rows).
     */
    template <>
    void to_json<Matrix4f>(nlohmann::json& j, const MatrixBase<Matrix4f>& vector);
    
    /**
     * @brief Reads a 4x4 matrix as written by to_json<Matrix4f>().
     * 
     * Reads the matrix as transformation matrix from position and orientation
     * or from list of rows, depending on j's contents.
     */
    template <>
    void from_json<Matrix4f>(const nlohmann::json& j, MatrixBase<Matrix4f>& vector);
    
    
    // Quaternion
    
    /// Writes the quaternion with qw, qx, qy, qz keys.
    template <typename Derived>
    void to_json(nlohmann::json& j, const QuaternionBase<Derived>& quat);
    
    /// Reads the quaternion from qw, qx, qy, qz keys.
    template <typename Derived>
    void from_json(const nlohmann::json& j, QuaternionBase<Derived>& quat);

    
    
    // IMPLEMENTATION

namespace json
{
    // "private" excplititly non-specialized implementation
    // (to make it callable from specialized implementations)
    // (anonymous namespace does not work because these functions are required
    // by specializations in the .cpp)

    /// Writes the matrix as list of rows.
    template <typename Derived>
    void to_json_base(nlohmann::json& j, const MatrixBase<Derived>& matrix)
    {
        for (int row = 0; row < matrix.rows(); ++row)
        {
            nlohmann::json column = nlohmann::json::array();
            for (int col = 0; col < matrix.cols(); ++col)
            {
                column.push_back(matrix(row, col));
            }
            j.push_back(column);
        }
    }
    
    /// Reads the matrix from list of rows.
    template <typename Derived>
    void from_json_base(const nlohmann::json& j, MatrixBase<Derived>& matrix)
    {
        using Scalar = typename MatrixBase<Derived>::Scalar;
        using Index = typename MatrixBase<Derived>::Index;
        
        for (std::size_t row = 0; row < j.size(); ++row)
        {
            const auto& jrow = j.at(row);
            for (std::size_t col = 0; col < jrow.size(); ++col)
            {
                const auto& value = jrow.at(col);
                matrix(static_cast<Index>(row), static_cast<Index>(col)) = value.get<Scalar>();
            }
        }
    }
}
  

    template <typename Derived>
    void to_json(nlohmann::json& j, const MatrixBase<Derived>& matrix)
    {
        to_json_base(j, matrix);
    }
    
    template <typename Derived>
    void from_json(const nlohmann::json& j, MatrixBase<Derived>& matrix)
    {
        from_json_base(j, matrix);
    }

    
    template <typename Derived>
    void to_json(nlohmann::json& j, const QuaternionBase<Derived>& quat)
    {
        j["qw"] = quat.w();
        j["qx"] = quat.x();
        j["qy"] = quat.y();
        j["qz"] = quat.z();
    }

    template <typename Derived>
    void from_json(const nlohmann::json& j, QuaternionBase<Derived>& quat)
    {
        using Scalar = typename QuaternionBase<Derived>::Scalar;
        quat.w() = j.at("qw").get<Scalar>();
        quat.x() = j.at("qx").get<Scalar>();
        quat.y() = j.at("qy").get<Scalar>();
        quat.z() = j.at("qz").get<Scalar>();
    }
    
}
