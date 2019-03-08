#include "eigen_conversion.h"


namespace Eigen
{
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
            json::to_json_base(j, matrix);
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
            json::from_json_base(j, matrix);
        }
    }
}
