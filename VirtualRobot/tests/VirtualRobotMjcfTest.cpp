/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMjcfTest

#include <VirtualRobot/VirtualRobotTest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <VirtualRobot/MJCF/Document.h>


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

BOOST_AUTO_TEST_SUITE(VirtualRobotMjcfTest)


BOOST_AUTO_TEST_CASE(test_boost_lexical_cast)
{
    for (bool in : { true, false })
    {
        const std::string string = boost::lexical_cast<std::string>(in);
        bool out = boost::lexical_cast<bool>(string);
        BOOST_TEST_MESSAGE(in << " -> '" << string << "' -> " << out);
        BOOST_CHECK_EQUAL(in, out);
    }
    
    // this cannot be handled by boost
    BOOST_CHECK_THROW(boost::lexical_cast<bool>("true"), boost::bad_lexical_cast);
    BOOST_CHECK_THROW(boost::lexical_cast<bool>("false"), boost::bad_lexical_cast);
}

BOOST_AUTO_TEST_SUITE_END()

using namespace Eigen;

BOOST_AUTO_TEST_CASE(test_attrib_conversion_vector3)
{
    Vector3f in(1, -3, 2.4f), out;
    
    const std::string string = mjcf::toAttr(in);
    mjcf::fromAttr(string, out);
    
    BOOST_CHECK_EQUAL(in, out);
}


struct Fixture
{
    mjcf::Document document;
    
    mjcf::Body body;
    mjcf::Joint joint;
    
    Fixture()
    {
        body = document.worldbody().addBody("thebody");
        joint = body.addJoint();
        joint.name = "thejoint";
    }
};


BOOST_FIXTURE_TEST_SUITE(VirtualRobotMjcfTest, Fixture)

BOOST_AUTO_TEST_CASE(test_attribute_pos)
{
    Eigen::Vector3f in(1, -3, 2.4f), out;
    body.pos = in;
    out = body.pos;
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_attribute_quat)
{
    Eigen::Quaternionf out, in(Eigen::AngleAxisf(1.2, Eigen::Vector3f(1, -3, 2.4f).normalized()));
    body.quat = in;
    out = body.quat;
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_SUITE_END()
