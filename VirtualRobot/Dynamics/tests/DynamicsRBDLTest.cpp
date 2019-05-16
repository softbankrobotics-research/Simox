/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_DynamicsRBDLTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Dynamics/Dynamics.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Eigen/Core>

#include <rbdl/SpatialAlgebraOperators.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <Tools/Gravity.h>

using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(DynamicsTests)

BOOST_AUTO_TEST_CASE(testRBDLTransformationOrientation)
{
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0*M_PI,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());

    // two types of transformations
    // 1) transformation between body/frame A and B
    // 2) transformation of position in frame A to frame B


    Model model;
    Joint jointA = Joint(JointTypeRevolute, Eigen::Vector3d::UnitZ());
    model.AppendBody(
                SpatialTransform(m.inverse(), // I have to use the rotation matrix for the frame transformation from A to B (Type 2) <------------
                                 Eigen::Vector3d(0,0,0)),
                     jointA, Body(), "bodyA");

    Joint jointB = Joint(JointTypeRevolute, Eigen::Vector3d::UnitZ());
    model.AppendBody(SpatialTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0)),
                     jointB, Body(), "bodyB");

    Eigen::Vector3d positionInBodyB = CalcBaseToBodyCoordinates(model, Eigen::Vector2d::Zero(), 1, Eigen::Vector3d(1,0,0));
    std::cout << "rotation: position in bodyB\n" << positionInBodyB << endl;

}

BOOST_AUTO_TEST_CASE(testRBDLTransformationTranslation)
{
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;
    // two types of transformations
    // 1) transformation between body/frame A and B
    // 2) transformation of position in frame A to frame B


    Model model;
    Joint jointA = Joint(JointTypeRevolute, Eigen::Vector3d::UnitZ());
    model.AppendBody(
                SpatialTransform(Eigen::Matrix3d::Identity(),
                                 Eigen::Vector3d(1,0,0)), // I have to use the translation for the transformation between body A and B (type 1) <--------------------
                     jointA, Body(), "bodyA");

    Joint jointB = Joint(JointTypeRevolute, Eigen::Vector3d::UnitZ());
    model.AppendBody(SpatialTransform(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0)),
                     jointB, Body(), "bodyB");

    Eigen::Vector3d positionInBodyB = CalcBaseToBodyCoordinates(model, Eigen::Vector2d::Zero(), 1, Eigen::Vector3d(0,0,0));
    std::cout << "translation: position in bodyB\n" << positionInBodyB << endl;

}

BOOST_AUTO_TEST_CASE(testRBDLConvertRobot)
{
    std::string robFile = "robots/ArmarIII/ArmarIII.xml";
    std::string rnsName = "LeftArm";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(robFile);
    BOOST_REQUIRE(fileOK);
    RobotPtr robot;
    BOOST_REQUIRE_NO_THROW(robot = RobotIO::loadRobot(robFile));
    BOOST_REQUIRE(robot);
    BOOST_REQUIRE(robot->hasRobotNodeSet(rnsName));
    RobotNodeSetPtr rns = robot->getRobotNodeSet(rnsName);

    // Check if Virtual Robot Result for gravity torque is the same as computated with RBDL
    Dynamics dynamics(rns, rns);
    Gravity gravity(robot, rns, rns);
    Eigen::VectorXd torque = dynamics.getGravityMatrix(Eigen::VectorXd::Zero(rns->getSize()));
    std::vector<float> floatVec(rns->getSize());
    gravity.computeGravityTorque(floatVec);
    Eigen::VectorXf torque2 = Eigen::Map<Eigen::VectorXf>(floatVec.data(), floatVec.size());
    std::cout << "gravity from RBDL:\n" << torque << std::endl;
    std::cout << "gravity from VR:\n" << torque2 << std::endl;
    BOOST_CHECK(torque.isApprox(torque2.cast<double>(), 1e-4));


}

BOOST_AUTO_TEST_CASE(testRBDLConvertSimpleRobot)
{
    std::string robFile = "robots/examples/rbdl-test-robot.xml";
    std::string rnsName = "Arm";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(robFile);
    BOOST_REQUIRE(fileOK);
    RobotPtr robot;
    BOOST_REQUIRE_NO_THROW(robot = RobotIO::loadRobot(robFile));
    BOOST_REQUIRE(robot);
    BOOST_REQUIRE(robot->hasRobotNodeSet(rnsName));
    RobotNodeSetPtr rns = robot->getRobotNodeSet(rnsName);

    auto base = robot->getRobotNode("Base");
    auto j1 = robot->getRobotNode("Joint1");
    Eigen::Vector3f vec(1,0,0);
    std::cout << "base to j1:\n" << MathTools::eigen4f2rpy(base->getTransformationTo(j1)) << endl;

    std::cout << "position\n" << vec << " in j1:\n" << j1->transformTo(base,vec) << endl;

    // Check if Virtual Robot Result for gravity torque is the same as computated with RBDL
    Dynamics dynamics(rns, rns);
    Gravity gravity(robot, rns, rns);
    Eigen::VectorXd torque = dynamics.getGravityMatrix(Eigen::VectorXd::Zero(rns->getSize()));
    std::vector<float> floatVec(rns->getSize());
    gravity.computeGravityTorque(floatVec);
    Eigen::VectorXf torque2 = Eigen::Map<Eigen::VectorXf>(floatVec.data(), floatVec.size());
    std::cout << "gravity from RBDL:\n" << torque << std::endl;
    std::cout << "gravity from VR:\n" << torque2 << std::endl;
    BOOST_CHECK(torque.isApprox(torque2.cast<double>(), 1e-4));


}

BOOST_AUTO_TEST_SUITE_END()
