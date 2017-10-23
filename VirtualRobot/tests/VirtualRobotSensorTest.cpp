/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotSensorTest
#include <VirtualRobot/VirtualRobotTest.h>

#include <VirtualRobot/Model/Nodes/Attachments/PositionSensor.h>
#include <VirtualRobot/Model/Nodes/Attachments/ForceTorqueSensor.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>

BOOST_AUTO_TEST_SUITE(Sensor)

BOOST_AUTO_TEST_CASE(testPositionSensor)
{
    const std::string robotString =
        "<Model Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Joint type='prismatic'>"
        "    <TranslationDirection x='0' y='0' z='1'/>"
        "    <Limits units='mm' lo='0' hi='1000'/>"
        "  </Joint>"
        "  <Sensor type='position' name='sensor1'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
        "  </Sensor>"
        "  <Child name='Link1'/>"
        " </RobotNode>"
        " <RobotNode name='Link1'>"
        "  <Sensor type='position' name='sensor2'>"
        "    <Transform>"
        "       <Translation x='0' y='0' z='100'/>"
        "    </Transfrom>"
        "  </Sensor>"
        " </RobotNode>"
        "</Model>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);
    VirtualRobot::ModelJointPtr rn = rob->getJoint("Joint1");
    BOOST_REQUIRE(rn);
    BOOST_REQUIRE(rob->hasSensor("sensor1"));
    BOOST_REQUIRE(rob->hasSensor("sensor2"));

    VirtualRobot::PositionSensorPtr ft1  = std::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rob->getSensor("sensor1"));
    VirtualRobot::PositionSensorPtr ft2  = std::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rob->getSensor("sensor2"));
    BOOST_REQUIRE(ft1);
    BOOST_REQUIRE(ft2);

    Eigen::Matrix4f p = ft1->getGlobalPose();
    Eigen::Matrix4f p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 0;

    BOOST_REQUIRE(p.isApprox(p2));

    rn->setJointValue(333.0f);
    p = ft1->getGlobalPose();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 333.0f;

    BOOST_REQUIRE(p.isApprox(p2));

    std::vector<VirtualRobot::PositionSensorPtr> posSensors = rob->getAttachments<VirtualRobot::PositionSensor>();
    BOOST_REQUIRE(posSensors.size()==2);
}

BOOST_AUTO_TEST_CASE(testFTSensor)
{
    const std::string robotString =
        "<Model Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Joint type='prismatic'>"
        "    <TranslationDirection x='0' y='0' z='1'/>"
        "    <Limits units='mm' lo='0' hi='1000'/>"
        "  </Joint>"
        "  <Sensor type='forcetorque' name='sensor1'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
        "  </Sensor>"
        "  <Child name='Link1'/>"
        " </RobotNode>"
        " <RobotNode name='Link1'>"
        "  <Sensor type='forcetorque' name='sensor2'>"
        "    <Transform>"
        "       <Translation x='0' y='0' z='100'/>"
        "    </Transfrom>"
        "  </Sensor>"
        " </RobotNode>"
        "</Model>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);
    VirtualRobot::ModelJointPtr rn = rob->getJoint("Joint1");
    BOOST_REQUIRE(rn);
    BOOST_REQUIRE(rob->hasSensor("sensor1"));
    BOOST_REQUIRE(rob->hasSensor("sensor2"));

    VirtualRobot::ForceTorqueSensorPtr ft1  = std::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(rob->getSensor("sensor1"));
    VirtualRobot::ForceTorqueSensorPtr ft2  = std::dynamic_pointer_cast<VirtualRobot::ForceTorqueSensor>(rob->getSensor("sensor2"));
    BOOST_REQUIRE(ft1);
    BOOST_REQUIRE(ft2);

    Eigen::Matrix4f p = ft1->getGlobalPose();
    Eigen::Matrix4f p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 0;

    BOOST_REQUIRE(p.isApprox(p2));

    rn->setJointValue(333.0f);
    p = ft1->getGlobalPose();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 333.0f;

    BOOST_REQUIRE(p.isApprox(p2));
}

BOOST_AUTO_TEST_SUITE_END()
