/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJsonEigenConversionTest

#include <VirtualRobot/VirtualRobotTest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <VirtualRobot/Util/json/eigen_conversion.h>


namespace Eigen
{
    bool operator==(const Quaternionf& lhs, const Quaternionf& rhs)
    {
        return lhs.isApprox(rhs, 0);
    }
    
    std::ostream& operator<<(std::ostream& os, const Quaternionf& rhs)
    {
        os << "[ " << rhs.w() << " | " << rhs.x() << " " << rhs.y() << " " << rhs.z() << " ]";
        return os;
    }
}


using namespace Eigen;


BOOST_AUTO_TEST_SUITE(VirtualRobotJsonEigenConversionTest)


BOOST_AUTO_TEST_CASE(test_matrix4f_non_transform)
{
    Matrix4f in = in.Identity(), out = out.Zero();
    
    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }
    
    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));
    
    out = j;
    BOOST_CHECK_EQUAL(in, out);
    
    out = j.get<Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_matrix4f_transform)
{
    Matrix4f in = math::Helpers::Pose(Vector3f { 3, 2, 3 },
                                      AngleAxisf( 1.2f, Eigen::Vector3f(1,2,3).normalized()));
    Matrix4f out = out.Zero();
    
    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));
    
    out = j;
    BOOST_CHECK_EQUAL(in, out);
    
    out = j.get<Matrix4f>();
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_CASE(test_vector3f)
{
    Vector3f in = in.Identity(), out = out.Zero();
    
    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }
    
    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));
    
    out = j;
    BOOST_CHECK_EQUAL(in, out);
    
    out = j.get<Vector3f>();
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_CASE(test_quaternionf)
{
    Quaternionf in { AngleAxisf(static_cast<float>(M_PI), Vector3f(1, 1, 1).normalized()) };
    Quaternionf out = out.Identity();
    
    nlohmann::json j;
    j = in;
    BOOST_TEST_MESSAGE("JSON: \n" << j.dump(2));
    
    // out = j; cannot be correctly resolved
    
    out = j.get<Quaternionf>();
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_SUITE_END()
