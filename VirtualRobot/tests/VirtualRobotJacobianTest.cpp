/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJacobianTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <algorithm>

BOOST_AUTO_TEST_SUITE(RobotNode)

#define MAX_ERROR 0.3f
#define STEP_SIZE 0.001f

using namespace VirtualRobot;

BOOST_AUTO_TEST_CASE(testJacobianRevoluteJoint)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"

        " <RobotNode name='Joint1'>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='1' y='0' z='0'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"

        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "     <Translation x='100' y='200' z='0'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='0' y='0' z='1'/>"
        "  </Joint>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"

        " <RobotNode name='Joint3'>"
        "    <Transform>"
        "     <Translation x='0' y='200' z='0'/>"
        "    </Transform>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    const std::string node3 = "Joint3";
    VirtualRobot::RobotNodePtr r1 = rob->getModelNode(node1);
    VirtualRobot::RobotNodePtr r2 = rob->getModelNode(node2);
    VirtualRobot::RobotNodePtr r3 = rob->getModelNode(node3);
    BOOST_REQUIRE(r1);
    BOOST_REQUIRE(r2);
    BOOST_REQUIRE(r3);

    std::vector< VirtualRobot::RobotNodePtr > nodes;
    nodes.push_back(r1);
    nodes.push_back(r2);
    nodes.push_back(r3);
    VirtualRobot::JointSetPtr kc(VirtualRobot::JointSet::createJointSet(rob, "KinChain", nodes, r1));
    BOOST_REQUIRE(kc);
    //BOOST_CHECK_EQUAL(kc->isKinematicChain(), true);

    VirtualRobot::RobotNodeSetPtr node_set;

    VirtualRobot::DifferentialIK ik(kc);
    Eigen::VectorXf jV(2);
    jV << 0.78f, 0.78f;
    kc->setJointValues(jV);

    // Calculate the Jacobi matrix at the given position
    Eigen::MatrixXf jacobian = ik.getJacobianMatrix(kc->getTCP());

    // Calculate the Differences quotient
    Eigen::Matrix4f a = r3->getGlobalPose();
    Eigen::MatrixXf DiffQuot(3, 2);
    jV << 0.78f + STEP_SIZE, 0.78f ;
    kc->setJointValues(jV);
    DiffQuot.block<3, 1>(0, 0) = (r3->getGlobalPose().block<3, 1>(0, 3) - a.block<3, 1>(0, 3)) / STEP_SIZE;
    jV << 0.78f, 0.78f + STEP_SIZE;
    kc->setJointValues(jV);
    DiffQuot.block<3, 1>(0, 1) = (r3->getGlobalPose().block<3, 1>(0, 3) - a.block<3, 1>(0, 3)) / STEP_SIZE;

    // Compare both and check if they are similar enough.
    //std::cout << "Jacobian:\n " << jacobian.block<3,2>(0,0) << std::endl;
    //std::cout << "Differential quotient:\n " << DiffQuot << std::endl;
    //std::cout << (  (jacobian.block<3,2>(0,0) -  DiffQuot).array().abs() < 0.2     ).all() << std::endl;
    BOOST_CHECK(((jacobian.block<3, 2>(0, 0) -  DiffQuot).array().abs() < MAX_ERROR).all());

}
/*
BOOST_AUTO_TEST_CASE(testJacobianRegularization)
{
    std::cout << "testJacobianRegularization" << std::endl;
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr robot = ModelIO::loadModel(filename);
    JointSetPtr rns = robot->getJointSet("RightArm");

    std::cout << "robot loaded" << std::endl;
    {

        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVDDamped);
        ik.setDampedSvdLambda(1);
        //ik.setPseudoInverseMMscaling(1);
        //ik.setPseudoInverseRadianScaling(1);
        Eigen::MatrixXf jacobi = ik.getJacobianMatrix(rns->getTCP());
        Eigen::MatrixXf invjac = ik.computePseudoInverseJacobianMatrix(jacobi, Eigen::VectorXf());
        Eigen::MatrixXf test = jacobi * invjac;

        Eigen::MatrixXf errMat = ik.getJacobiRegularization().asDiagonal() * (test - Eigen::MatrixXf::Identity(6, 6));
        std::cout << test << std::endl;
        std::cout << errMat << std::endl;

        float error = errMat.norm();
        std::cout << "error: " << error << std::endl;
        //check that error is big for SVDDamped when jacobi is not regularized and lambda is too big
        BOOST_CHECK_GE(error, 0.1);
    }
    {

        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVDDamped);
        Eigen::MatrixXf jacobi = ik.getJacobianMatrix(rns->getTCP());
        Eigen::MatrixXf invjac = ik.computePseudoInverseJacobianMatrix(jacobi, ik.getJacobiRegularization());
        Eigen::MatrixXf test = jacobi * invjac;

        Eigen::MatrixXf errMat = ik.getJacobiRegularization().asDiagonal() * (test - Eigen::MatrixXf::Identity(6, 6));
        std::cout << test << std::endl;
        std::cout << errMat << std::endl;

        float error = errMat.norm();
        std::cout << "error: " << error << std::endl;
        //check that error is small when radians are scaled up
        BOOST_CHECK_LE(error, 0.1);
        //check that error is still existant for SVDDamped
        BOOST_CHECK_GE(error, 1e-3);
    }
    {
        //robot->setRadianToMMfactor(10);

        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVD);
        Eigen::MatrixXf jacobi = ik.getJacobianMatrix(rns->getTCP());
        Eigen::MatrixXf invjac = ik.computePseudoInverseJacobianMatrix(jacobi, ik.getJacobiRegularization());
        Eigen::MatrixXf test = jacobi * invjac;
        std::cout << test << std::endl;

        float error = (test - Eigen::MatrixXf::Identity(6, 6)).norm();
        std::cout << "error: " << error << std::endl;
        //check that error is very small for SVD
        BOOST_CHECK_LE(error, 0.01);
    }
}

BOOST_AUTO_TEST_CASE(testJacobianSingularity)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr robot = ModelIO::loadModel(filename);
    JointSetPtr rns = robot->getJointSet("RightArm");

    std::cout << "robot loaded" << std::endl;

    Eigen::VectorXf initialJointAngles = rns->getJointValuesEigen();

    Eigen::VectorXf vel(6);
    vel << 0, 10, 0, 0, 0, 0;

    {
        rns->setJointValues(initialJointAngles);
        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVD);
        float maxError = 0;
        for(int i = 0; i < 40; i++)
        {
            Eigen::MatrixXf jacobi = ik.getJacobianMatrix(rns->getTCP());
            Eigen::MatrixXf invjac = ik.computePseudoInverseJacobianMatrix(jacobi, ik.getJacobiRegularization());
            Eigen::VectorXf jointVel = invjac * vel;
            //std::cout << jacobi << std::endl;
            //std::cout << (jacobi * jointVel).transpose() << std::endl;
            //std::cout << rns->getTCP()->getPositionInRootFrame().transpose() << std::endl;
            Eigen::Vector3f oldPos = rns->getTCP()->getPositionInRootFrame();
            rns->setJointValues(rns->getJointValuesEigen() + jointVel);
            //std::cout << (rns->getTCP()->getPositionInRootFrame() - oldPos).transpose() << std::endl;
            Eigen::Vector3f diff = rns->getTCP()->getPositionInRootFrame() - oldPos;
            //std::cout << (vel.topRows(3) - diff).norm() << " " <<  diff.norm() << std::endl;
            maxError = std::max(maxError, diff.norm());
            //std::cout << jointVel.transpose() << std::endl;
        }
        std::cout << "maxError: " << maxError << std::endl;
        BOOST_CHECK_GE(maxError, 50);

    }
    std::cout << "#### eSVDDamped" << std::endl;
    {
        rns->setJointValues(initialJointAngles);
        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVDDamped);
        ik.setDampedSvdLambda(0.2);
        float maxError = 0;
        for(int i = 0; i < 40; i++)
        {
            Eigen::MatrixXf jacobi = ik.getJacobianMatrix(rns->getTCP());
            Eigen::MatrixXf invjac = ik.computePseudoInverseJacobianMatrix(jacobi, ik.getJacobiRegularization());
            Eigen::VectorXf jointVel = invjac * vel;
            //std::cout << (jacobi * jointVel).transpose() << std::endl;
            //std::cout << rns->getTCP()->getPositionInRootFrame().transpose() << std::endl;
            Eigen::Vector3f oldPos = rns->getTCP()->getPositionInRootFrame();
            rns->setJointValues(rns->getJointValuesEigen() + jointVel);
            //std::cout << (rns->getTCP()->getPositionInRootFrame() - oldPos).norm() << " " << (rns->getTCP()->getPositionInRootFrame() - oldPos).transpose() << std::endl;
            Eigen::Vector3f diff = rns->getTCP()->getPositionInRootFrame() - oldPos;
            //std::cout << (vel.topRows(3) - diff).norm() << " " <<  diff.norm() << std::endl;
            maxError = std::max(maxError, diff.norm());
            //std::cout << (rns->getTCP()->getPositionInRootFrame() - oldPos).transpose() << std::endl;
            //std::cout << jointVel.transpose() << std::endl;
        }
        std::cout << "maxError: " << maxError << std::endl;
        BOOST_CHECK_LE(maxError, 50);
    }

    /*std::cout << "#### eSVDDamped + calculateJointVelocity" << std::endl;
    {
        rns->setJointValues(initialJointAngles);
        VirtualRobot::DifferentialIK ik(rns, RobotNodePtr(), JacobiProvider::eSVDDamped);
        for(int i = 0; i < 40; i++)
        {
            Eigen::VectorXf jointVel = ik.calculateJointVelocity(rns->getTCP(), vel);
            Eigen::Vector3f oldPos = rns->getTCP()->getPositionInRootFrame();
            rns->setJointValues(rns->getJointValuesEigen() + jointVel);
            std::cout << (rns->getTCP()->getPositionInRootFrame() - oldPos).transpose() << std::endl;
        }

    }
}*/

BOOST_AUTO_TEST_SUITE_END()
