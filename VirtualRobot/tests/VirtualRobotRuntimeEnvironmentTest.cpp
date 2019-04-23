/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotRuntimeEnvironmentTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/RuntimeEnvironment.h>


BOOST_AUTO_TEST_SUITE(RuntimeEnvironment)


BOOST_AUTO_TEST_CASE(test_toVector3f_valid_input)
{
    Eigen::Vector3f vector;
            
    BOOST_CHECK(VirtualRobot::RuntimeEnvironment::toVector3f("(0, 0, 0)", vector));
    BOOST_CHECK_EQUAL(vector, Eigen::Vector3f::Zero());
    
    BOOST_CHECK(VirtualRobot::RuntimeEnvironment::toVector3f("(1, 2, 3)", vector));
    BOOST_CHECK_EQUAL(vector, Eigen::Vector3f(1, 2, 3));
    
    BOOST_CHECK(VirtualRobot::RuntimeEnvironment::toVector3f("(-1, 2, 5)", vector));
    BOOST_CHECK_EQUAL(vector, Eigen::Vector3f(-1, 2, 5));
    
    BOOST_CHECK(VirtualRobot::RuntimeEnvironment::toVector3f("(-3.14, 0, 9.99)", vector));
    BOOST_CHECK_EQUAL(vector, Eigen::Vector3f(-3.14f, 0, 9.99f));
}


BOOST_AUTO_TEST_CASE(test_toVector3f_invalid_input)
{
    Eigen::Vector3f vector;
            
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("()", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("nonsense", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(0)", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(0, 0)", vector));
    
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("1, 2, 3", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("1, 2, 3)", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(1, 2, 3", vector));
    
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(0. 0, 0)", vector));
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(0; 0; 0)", vector));
    
    BOOST_CHECK(!VirtualRobot::RuntimeEnvironment::toVector3f("(0, 0, a)", vector));
}


BOOST_AUTO_TEST_SUITE_END()
