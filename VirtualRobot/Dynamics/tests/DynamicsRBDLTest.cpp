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
#include <Tools/Gravity.h>

using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(DynamicsTests)

BOOST_AUTO_TEST_CASE(testRBDLTransformation)
{
    using namespace Eigen;
    using namespace RigidBodyDynamics::Math;
    Eigen:: Matrix3d m;
    m = AngleAxisd(0.0*M_PI, Eigen::Vector3d::UnitX())
      * AngleAxisd(0.0*M_PI,  Eigen::Vector3d::UnitY())
      * AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d vec1 = Eigen::Vector3d(1,0,0);
    SpatialTransform transRotInv(m.inverse(), Eigen::Vector3d(0,0,0));
    SpatialTransform transRot(m, Eigen::Vector3d(0,0,0));
    SpatialTransform transTranslation(Eigen::Matrix3d::Identity(), vec1);
    std::cout << "Spatial Transform:\n" << (transTranslation*transRot).r << std::endl;;
    std::cout << "Spatial Transform from inverse:\n" << (transTranslation*transRotInv).r << std::endl;;
    std::cout << "normal rotation matrix:\n" << m * vec1 << std::endl;
    BOOST_CHECK((m * vec1).isApprox((transTranslation*transRotInv).r));
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

BOOST_AUTO_TEST_SUITE_END()
