/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJsonEigenConversionTest

#include <VirtualRobot/VirtualRobotTest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <VirtualRobot/Util/json/eigen_conversion.hpp>

#if BOOST_VERSION > 105300
#define BOOST_MESSAGE(msg) BOOST_TEST_MESSAGE(msg)
#endif


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


using json = nlohmann::json;
using namespace Eigen;


BOOST_AUTO_TEST_SUITE(VirtualRobotJsonEigenConversionTest);


BOOST_AUTO_TEST_CASE(test_matrix)
{
    Matrix4f in = in.Identity(), out = out.Zero();
    
    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }
    
    json j;
    j = in;
    out = j;
    BOOST_MESSAGE("JSON: \n" << j.dump(2));
    
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_CASE(test_vector)
{
    Vector3f in = in.Identity(), out = out.Zero();
    
    for (int i = 0; i < in.size(); ++i)
    {
        in(i) = i;
    }
    
    json j;
    j = in;
    out = j;
    BOOST_MESSAGE("JSON: \n" << j.dump(2));
    
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_CASE(test_quaternion)
{
    Quaternionf in { AngleAxisf(static_cast<float>(M_PI), Vector3f(1, 1, 1).normalized()) };
    Quaternionf out = out.Identity();
    
    json j;
    j = in;
    out = j.get<Quaternionf>();
    BOOST_MESSAGE("JSON: \n" << j.dump(2));
    
    BOOST_CHECK_EQUAL(in, out);
}


BOOST_AUTO_TEST_SUITE_END()
