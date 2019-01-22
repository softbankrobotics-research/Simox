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


BOOST_AUTO_TEST_SUITE(MathHelpers)


BOOST_AUTO_TEST_CASE(test_CwiseMin_CwiseMax)
{
    Eigen::Vector3f a (-1, 3, 5), b(0, 3, 1);
    Eigen::Vector3f min (-1, 3, 1);
    Eigen::Vector3f max (0, 3, 5);
    BOOST_CHECK_EQUAL(Helpers::CwiseMin(a, b), min);
    BOOST_CHECK_EQUAL(Helpers::CwiseMax(a, b), max);
}

BOOST_AUTO_TEST_CASE(test_CwiseDivide)
{
    Eigen::Vector3f a (0, 5, -9), b(10, 2, 3);
    Eigen::Vector3f quot (0, 2.5, -3);
    BOOST_CHECK_EQUAL(Helpers::CwiseDivide(a, b), quot);
}

BOOST_AUTO_TEST_CASE(test_Swap)
{
    float a = 5, b = -10;
    Helpers::Swap(a, b);
    BOOST_CHECK_EQUAL(a, -10);
    BOOST_CHECK_EQUAL(b, 5);
}

BOOST_AUTO_TEST_CASE(test_GetRotationMatrix)
{
    Eigen::Vector3f source(1, 2, 3), target(-3, 2, 5);  // not normalized
    Eigen::Matrix3f matrix = Helpers::GetRotationMatrix(source, target);
    
    BOOST_CHECK((matrix * matrix.transpose()).isIdentity(1e-6f));
    BOOST_CHECK((matrix * source.normalized()).isApprox(target.normalized(), 1e-6f));
}

BOOST_AUTO_TEST_CASE(test_TransformPosition)
{
    Eigen::Vector3f vector(1, 2, 3);

    Eigen::Vector3f translation(4, 5, 6);
    Eigen::AngleAxisf rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY());
    
    Eigen::Matrix4f transform = transform.Identity();
    
    // identity
    transform.setIdentity();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), 
                      vector);
    
    // translation only
    transform.setIdentity();
    Helpers::posBlock(transform) = translation;
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), 
                      vector + translation);
    
    // rotation only
    transform.setIdentity();
    Helpers::oriBlock(transform) = rotation.toRotationMatrix();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), 
                      rotation * vector);

    // full transform
    transform.setIdentity();
    Helpers::posBlock(transform) = translation;
    Helpers::oriBlock(transform) = rotation.toRotationMatrix();
    BOOST_CHECK_EQUAL(Helpers::TransformPosition(transform, vector), 
                      rotation * vector + translation);
}


BOOST_AUTO_TEST_CASE(test_InvertPose)
{
    Eigen::Vector3f translation(4, 5, 6);
    Eigen::AngleAxisf rotation(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitY());
    
    Eigen::Matrix4f pose = Helpers::toPose(translation, rotation);
    Eigen::Matrix4f inv;
    
    // in-place
    inv = pose;
    Helpers::InvertPose(inv);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));
    
    // returned
    inv.setIdentity();
    inv = Helpers::InvertedPose(pose);
    BOOST_CHECK((pose * inv).isIdentity(1e-6f));
    BOOST_CHECK((inv * pose).isIdentity(1e-6f));
}


BOOST_AUTO_TEST_SUITE_END()



struct BlockFixture
{
    BlockFixture()
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
    Vector3f pos, pos2;
    Matrix3f ori, ori2;
    Quaternionf quat, quat2;
};


BOOST_FIXTURE_TEST_SUITE(MathHelpers, BlockFixture)

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
