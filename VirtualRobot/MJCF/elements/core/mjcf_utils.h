#pragma once

#include <string>
#include <type_traits>

#include <Eigen/Eigen>
#include <boost/lexical_cast.hpp>

#include "exceptions.h"


namespace std
{
    bool operator==(const char* lhs, const string& rhs);
    bool operator==(const string& lhs, const char* rhs);
}

namespace Eigen
{
    using Vector5f = Eigen::Matrix<float, 5, 1>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;
    using Vector7f = Eigen::Matrix<float, 7, 1>;

    template<typename Derived>
    struct is_matrix_expression : 
            std::is_base_of<Eigen::MatrixBase<typename std::decay<Derived>::type>, typename std::decay<Derived>::type>
    {};
    
    template<typename Derived>
    struct is_quaternion_expression : 
            std::is_base_of<Eigen::QuaternionBase<typename std::decay<Derived>::type>, typename std::decay<Derived>::type>
    {};
    
    template<typename Derived>
    struct is_eigen_expression : 
            is_matrix_expression<Derived>, is_quaternion_expression<Derived>
    {};
} 


namespace mjcf
{

    // Get lenght of common prefix of two strings (was used for mergin body names).
    std::size_t commonPrefixLength(const std::string& a, const std::string& b);

    bool equals(const char* lhs, const char* rhs);
    
    
    // VALUE -> ATTRIBUTE
    
    // Convert to MJCF XML attribute format.
    
    template <typename AttrT,
              typename std::enable_if<!Eigen::is_eigen_expression<AttrT>::value, AttrT>::type* = nullptr>
    std::string toAttr(const AttrT& b);
    template <>
    std::string toAttr<bool>(const bool& b);
    template <typename Derived>
    std::string toAttr(const Eigen::MatrixBase<Derived>& mat);
    template <typename Derived>
    std::string toAttr(const Eigen::QuaternionBase<Derived>& quat);


    // ATTRIBUTE -> VALUE
    
    // Convert from MJCF XML attribute.
    Eigen::Vector2f strToVec2(const char* string);
    Eigen::Vector3f strToVec3(const char* string);
    Eigen::Quaternionf strToQuat(const char* string);
    
    
    
    /// Single values via boost::lexical cast. Only enabled for non-Eigen types.
    template <typename AttrT,
              typename std::enable_if<!Eigen::is_eigen_expression<AttrT>::value, AttrT>::type* = nullptr>
    void fromAttr(const std::string& valueStr, AttrT& value);
    
    /// Bool
    void fromAttr(const std::string& valueStr, bool& value);
    
    /// Eigen Matrix and vectors
    template <typename Derived> 
    void fromAttr(const std::string& valueStr, Eigen::MatrixBase<Derived>& value);
    /// Eigen Quaternions
    template <typename Derived> 
    void fromAttr(const std::string& valueStr, Eigen::QuaternionBase<Derived>& value);

    
    
    template <typename Scalar>
    std::vector<Scalar> parseCoeffs(const std::string& string, char delim = ' ');
    
    
    // DEFINITIONS of templated methods

    template <typename AttrT,
              typename std::enable_if<!Eigen::is_eigen_expression<AttrT>::value, AttrT>::type*>
    std::string toAttr(const AttrT& b)
    {
        static_assert (!Eigen::is_eigen_expression<AttrT>::value, "Resolved an Eigen type.");
        static_assert (!std::is_same<bool, AttrT>::value, "Resolved bool.");
        return boost::lexical_cast<std::string>(b);
    }
    
    template<typename Derived>
    std::string toAttr(const Eigen::MatrixBase<Derived>& mat)
    {
        static const Eigen::IOFormat iof 
        {
            Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", ""
        };
    
        std::stringstream ss;
        ss << mat.format(iof);
        return ss.str();
    }
    
    template<typename Derived>
    std::string toAttr(const Eigen::QuaternionBase<Derived>& quat)
    {
        std::stringstream ss;
        ss << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
        return ss.str();
    }

    
    template <typename AttrT,
              typename std::enable_if<!Eigen::is_eigen_expression<AttrT>::value, AttrT>::type*>
    void fromAttr(const std::string& valueStr, AttrT& value)
    {
        static_assert (!Eigen::is_eigen_expression<AttrT>::value, "Resolved an Eigen type.");
        value = boost::lexical_cast<AttrT>(valueStr);
    }
    
    template <typename Derived> 
    void fromAttr(const std::string& valueStr, Eigen::MatrixBase<Derived>& value)
    {
        using Matrix = Eigen::MatrixBase<Derived>;
        using Scalar = typename Matrix::Scalar;
        
        std::vector<Scalar> coeffs;
        try 
        {
            coeffs = parseCoeffs<Scalar>(valueStr);
        }
        catch (const std::bad_cast& e) 
        {
            throw mjcf::ParseAttributeError(valueStr, typeid(Matrix), e.what());
        }
        
        if (value.size() >= 0 && static_cast<long>(coeffs.size()) != value.size())
        {
            throw mjcf::ParseAttributeError(valueStr, typeid(Matrix), "Number of coefficients does not match.");
        }
        
        long i = 0;
        for (const auto& coeff : coeffs)
        {
            value(i++) = coeff;
        }
    }
    
    template <typename Derived> 
    void fromAttr(const std::string& valueStr, Eigen::QuaternionBase<Derived>& value)
    {
        using Quaternion = Eigen::QuaternionBase<Derived>;
        using Scalar = typename Quaternion::Scalar;
        std::vector<Scalar> coeffs;
        
        try 
        {
            coeffs = parseCoeffs<Scalar>(valueStr);
        }
        catch (const std::bad_cast& e) 
        {
            throw mjcf::ParseAttributeError(valueStr, typeid(Quaternion), e.what());
        }
        
        
        if (coeffs.size() != 4)
        {
            throw mjcf::ParseAttributeError(valueStr, typeid(Quaternion),
                                            "Number of coefficients does not match.");
        }
        
        value.w() = coeffs[0];
        value.x() = coeffs[1];
        value.y() = coeffs[2];
        value.z() = coeffs[3];
    }
    
    
    template <typename Scalar>
    std::vector<Scalar> parseCoeffs(const std::string& string, char delim)
    {
        std::vector<Scalar> coeffs;

        std::stringstream ss(string);
        for (std::string valueStr; std::getline(ss, valueStr, delim);)
        {
            // may throw boost::bad_lexical_cast (derives from std::bad_cast)
            Scalar value = boost::lexical_cast<Scalar>(valueStr);
            coeffs.push_back(value);
        }

        return coeffs;
    }
}

