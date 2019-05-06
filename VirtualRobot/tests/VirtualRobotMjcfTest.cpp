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
    std::ostream& operator<<(std::ostream& os, const Vector3f& rhs)
    {
        static const IOFormat iof(4, 0, " ", " ", "", "", "[", "]");
        os << rhs.format(iof);
        return os;
    }

    bool operator==(const Quaternionf& lhs, const Quaternionf& rhs)
    {
        return lhs.isApprox(rhs);
    }
    
    std::ostream& operator<<(std::ostream& os, const Quaternionf& rhs)
    {
        os << "[ " << rhs.w() << " | " << rhs.x() << " " << rhs.y() << " " << rhs.z() << " ]";
        return os;
    }
}

#define MSG_CONVERSION(in, string, out) \
    BOOST_TEST_MESSAGE(in << " -> '" << string << "' -> " << out)

BOOST_AUTO_TEST_SUITE(VirtualRobotMjcfTest)

BOOST_AUTO_TEST_CASE(test_boost_lexical_cast)
{
    for (bool in : { true, false })
    {
        const std::string string = boost::lexical_cast<std::string>(in);
        bool out = boost::lexical_cast<bool>(string);
        MSG_CONVERSION(in, string, out);
        BOOST_CHECK_EQUAL(in, out);
    }
    
    // this cannot be handled by boost
    BOOST_CHECK_THROW(boost::lexical_cast<bool>("true"), boost::bad_lexical_cast);
    BOOST_CHECK_THROW(boost::lexical_cast<bool>("false"), boost::bad_lexical_cast);
}

BOOST_AUTO_TEST_SUITE_END()

using namespace Eigen;


BOOST_AUTO_TEST_CASE(test_parseCoeffs)
{
    const std::string string = "1 -3 2.4";
    Eigen::Vector3f vector(1, -3, 2.4f);
    
    std::vector<float> coeffs = mjcf::parseCoeffs<float>(string, ' ');
    
    BOOST_CHECK_EQUAL(coeffs[0], vector.x());
    BOOST_CHECK_EQUAL(coeffs[1], vector.y());
    BOOST_CHECK_EQUAL(coeffs[2], vector.z());
}


BOOST_AUTO_TEST_CASE(test_attrib_conversion_vector3f)
{
    Vector3f in(1, -3, 2.4f), out;
    
    const std::string string = mjcf::toAttr(in);
    mjcf::fromAttr(string, out);
    MSG_CONVERSION(in, string, out);
    
    BOOST_CHECK_EQUAL(in, out);
}

BOOST_AUTO_TEST_CASE(test_attrib_conversion_quaternionf)
{
    Quaternionf in(AngleAxisf(1.4f, Vector3f(.5, -1, .3f).normalized())), out;
    
    const std::string string = mjcf::toAttr(in);
    mjcf::fromAttr(string, out);
    MSG_CONVERSION(in, string, out);
    
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
