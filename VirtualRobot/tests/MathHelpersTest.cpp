/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_MathHelpersTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/math/Helpers.h>
#include <string>
#include <stdio.h>


using namespace Eigen;
using Helpers = math::Helpers;


struct Fixture
{
    Fixture()
    {
        quat = Quaternionf{
            AngleAxisf(static_cast<float>(M_PI), Vector3f::UnitZ()) 
            * AngleAxisf(static_cast<float>(M_PI_2), Vector3f::UnitY())
        };
        
        quat2 = AngleAxisf(static_cast<float>(M_PI_4), Vector3f::UnitX()) * quat;
        
        pos = Vector3f{ 1, 2, 3 };
        pos2 = Vector3f{ 4, 5, 6 };
        
        ori = quat.toRotationMatrix();
        ori2 = quat2.toRotationMatrix();
        
        pose.setIdentity();
        pose.block<3, 1>(0, 3) = pos;
        pose.block<3, 3>(0, 0) = ori;
    }
    
    
    Matrix4f pose;
    
    Vector3f pos;
    Matrix3f ori;
    
    Vector3f pos2;
    Matrix3f ori2;
    
    Quaternionf quat;
    Quaternionf quat2;
};


BOOST_FIXTURE_TEST_SUITE(MathHelpers, Fixture)

using namespace math;


BOOST_AUTO_TEST_CASE(test_posBlock_const)
{
    BOOST_CHECK_EQUAL(Helpers::posBlock(const_cast<const Matrix4f&>(pose)), pos);
}

BOOST_AUTO_TEST_CASE(test_posBlock_nonconst)
{
    BOOST_CHECK_EQUAL(Helpers::posBlock(pose), pos);
    
    Helpers::posBlock(pose) = pos2;
    BOOST_CHECK_EQUAL(Helpers::posBlock(pose), pos2);
}


BOOST_AUTO_TEST_CASE(test_oriBlock_const)
{
    BOOST_CHECK_EQUAL(Helpers::oriBlock(const_cast<const Eigen::Matrix4f&>(pose)), ori);
}

BOOST_AUTO_TEST_CASE(test_oriBlock_nonconst)
{
    BOOST_CHECK_EQUAL(Helpers::oriBlock(pose), ori);
    
    Helpers::oriBlock(pose) = ori2;
    BOOST_CHECK_EQUAL(Helpers::oriBlock(pose), ori2);
}


BOOST_AUTO_TEST_CASE(test_toPose_matrix_and_quaternion)
{
    BOOST_CHECK_EQUAL(Helpers::toPose(pos, quat), pose);
}

BOOST_AUTO_TEST_CASE(test_toPose_matrix_and_rotation_matrix)
{
    BOOST_CHECK_EQUAL(Helpers::toPose(pos, ori), pose);
}



BOOST_AUTO_TEST_SUITE_END()
