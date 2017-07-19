/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotConfigTest

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Model/ModelConfig.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(Scene)

BOOST_AUTO_TEST_CASE(testRobotConfigInvalidCreation)
{
    BOOST_REQUIRE_THROW(VirtualRobot::RobotConfigPtr c(new VirtualRobot::RobotConfig(VirtualRobot::RobotPtr(), "")), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotConfigSetConfig)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
		"        <Joint type='revolute'>                                                                              "
		"            <axis x='0' y='0' z='1'/>                                                                        "
		"            <Limits unit='degree' lo='-45' hi='45'/>                                                         "
		"        </Joint>                                                                                             "
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

	const std::string node1 = "Joint1";

	VirtualRobot::ModelJointPtr j;
	BOOST_REQUIRE_NO_THROW(j = rob->getJoint(node1));
	BOOST_REQUIRE(j);

	float jv = -100.0f;
	BOOST_REQUIRE_NO_THROW(jv = j->getJointValue());
	BOOST_REQUIRE(jv==0.0f);

	BOOST_REQUIRE_NO_THROW(j->setJointValue(0.1f));
	BOOST_REQUIRE_NO_THROW(jv = j->getJointValue());
	BOOST_REQUIRE(fabs(jv-0.1f) < 0.00001f);

    VirtualRobot::RobotConfigPtr c;
    BOOST_REQUIRE_NO_THROW(c.reset(new VirtualRobot::RobotConfig(rob, "test")));
    BOOST_REQUIRE(c);

	BOOST_REQUIRE_NO_THROW(c->setConfig(node1, 0.2f));
	BOOST_REQUIRE_NO_THROW(rob->setConfig(c));

	BOOST_REQUIRE_NO_THROW(jv = j->getJointValue());
	BOOST_REQUIRE(fabs(jv - 0.2f) < 0.00001f);
}



BOOST_AUTO_TEST_CASE(testRobotConfigSetInvalidConfig)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::ModelConfigPtr c;
    BOOST_REQUIRE_NO_THROW(c.reset(new VirtualRobot::ModelConfig(rob, "test")));
    BOOST_REQUIRE(c);

    const std::string node2 = "JointNotPresent";
    bool ok = c->setConfig(node2, 0.0f);
    BOOST_REQUIRE(!ok);
}



BOOST_AUTO_TEST_SUITE_END()
