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
 * json j;
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
    

    // MatrixBase

    template <typename Derived>
    void to_json(nlohmann::json& j, const MatrixBase<Derived>& matrix);
    
    template <typename Derived>
    void from_json(const nlohmann::json& j, MatrixBase<Derived>& matrix);
    

    // Specialization for Vector3f
    
    template <>
    void to_json<Vector3f>(nlohmann::json& j, const MatrixBase<Vector3f>& vector);
    
    template <>
    void from_json<Vector3f>(const nlohmann::json& j, MatrixBase<Vector3f>& vector);
    
    
    // Specialization for Matrix4f as Transformation matrix
    
    template <>
    void to_json<Matrix4f>(nlohmann::json& j, const MatrixBase<Matrix4f>& vector);
    
    template <>
    void from_json<Matrix4f>(const nlohmann::json& j, MatrixBase<Matrix4f>& vector);
    
    
    // Quaternion
    
    template <typename Derived>
    void to_json(nlohmann::json& j, const QuaternionBase<Derived>& quat);
    
    template <typename Derived>
    void from_json(const nlohmann::json& j, QuaternionBase<Derived>& quat);

    
    
    // IMPLEMENTATION

namespace
{
    // private excplititly non-specialized implementation
    // (to make it callable from specialized implementations)

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

    
    template <>
    void to_json<Vector3f>(nlohmann::json& j, const MatrixBase<Vector3f>& vector)
    {
        j["x"] = vector.x();
        j["y"] = vector.y();
        j["z"] = vector.z();
    }
    
    template <>
    void from_json<Vector3f>(const nlohmann::json& j, MatrixBase<Vector3f>& vector)
    {
        vector.x() = j.at("x").get<float>();
        vector.y() = j.at("y").get<float>();
        vector.z() = j.at("z").get<float>();
    }
    

    template <>
    void to_json<Matrix4f>(nlohmann::json& j, const MatrixBase<Matrix4f>& matrix)
    {
        // check lower row for [ 0 0 0 1 ]
        if (matrix.row(3).isApprox(Eigen::Vector4f(0, 0, 0, 1).transpose(), 1e-9f))
        {
            // must construct a Vector3f to trigger specialized version of to_json()
            j["pos"] = Eigen::Vector3f(math::Helpers::Position(matrix));
            j["ori"] = math::Helpers::Orientation(matrix);
        }
        else
        {
            to_json_base(j, matrix);
        }
    }
    
    template <>
    void from_json<Matrix4f>(const nlohmann::json& j, MatrixBase<Matrix4f>& matrix)
    {
        if (j.count("pos") > 0 && j.count("ori") > 0)
        {
            matrix = math::Helpers::Pose(j.at("pos").get<Eigen::Vector3f>(), 
                                         j.at("ori").get<Eigen::Matrix3f>());
        }
        else
        {
            from_json_base(j, matrix);
        }
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
