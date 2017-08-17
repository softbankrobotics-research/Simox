/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotIOTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
//#include <VirtualRobot/Nodes/Sensor.h>
//#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Model/ManipulationObject.h>
#include <string>


using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(VirtualRobotIO)


BOOST_AUTO_TEST_CASE(testRobotModelFromString)
{
    const std::string robotDef1 =
            "<?xml version='1.0' encoding='UTF-8'?>"
            "<ModelDescription>"
            "</ModelDescription>";

    const std::string robotDef2 =
            "<?xml version='1.0' encoding='UTF-8'?>"
            "<ModelDescription>"
            "    <URDF>urdf/Armar4.urdf</URDF>"
            "</ModelDescription>";

    std::string basePath = "robots/Armar4";

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = RobotIO::createRobotModelFromString(robotDef1,basePath));
    BOOST_REQUIRE(!r);

    BOOST_REQUIRE_NO_THROW(r = RobotIO::createRobotModelFromString(robotDef2,basePath));
    BOOST_REQUIRE(r);

    std::vector<RobotNodePtr> rn = r->getModelNodes();
    BOOST_REQUIRE_GT(rn.size(), 0);

    std::vector<ModelJointPtr> joints = r->getJoints();
    BOOST_REQUIRE_GT(joints.size(), 0);

    std::vector<ModelLinkPtr> links = r->getLinks();
    BOOST_REQUIRE_GT(links.size(), 0);

    std::vector<EndEffectorPtr> eefs = r->getEndEffectors();
    BOOST_REQUIRE_EQUAL(eefs.size(), 0);

    std::vector<RobotNodeSetPtr> rns = r->getModelNodeSets();
    BOOST_REQUIRE_EQUAL(rns.size(), 0);
}

/*
BOOST_AUTO_TEST_CASE(testRobotLoadXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = SimoxXMLFactory::loadRobotSimoxXML(filename));
    BOOST_REQUIRE(r);

    std::vector<RobotNodePtr> rn = r->getModelNodes();
    BOOST_REQUIRE_GT(rn.size(), 0);

    std::vector<EndEffectorPtr> eefs = r->getEndEffectors();
    BOOST_REQUIRE_GT(eefs.size(), 0);

    std::vector<RobotNodeSetPtr> rns = r->getModelNodeSets();
    BOOST_REQUIRE_GT(rns.size(), 0);
}

BOOST_AUTO_TEST_CASE(testRobotSaveXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = SimoxXMLFactory::loadRobotSimoxXML(filename));
    BOOST_REQUIRE(r);

    boost::system::error_code ec;
    boost::filesystem::path tempDir = boost::filesystem::temp_directory_path(ec);
    BOOST_REQUIRE(ec.value() == boost::system::errc::success);

    boost::filesystem::path robName("ArmarIII_tmp.xml");
    boost::filesystem::path filenameTmp = boost::filesystem::operator/(tempDir, robName);

    bool saveOK;
    BOOST_REQUIRE_NO_THROW(saveOK = SimoxXMLFactory::saveXML(r, robName.string(), tempDir.string()));
    BOOST_REQUIRE(saveOK);

    //reload robot
    RobotPtr r2;
    BOOST_REQUIRE_NO_THROW(r2 = SimoxXMLFactory::loadRobotSimoxXML(filenameTmp.string()));
    BOOST_REQUIRE(r2);
}

BOOST_AUTO_TEST_CASE(testLoadStoreManipulationObjectPhysics)
{
    std::string filename("objects/physics-test.xml");
    bool fileOK = VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    ManipulationObjectPtr manipulatioObject = ObjectIO::loadManipulationObject(filename);

    ModelLink::Physics physicsObject = manipulatioObject->getLinks().at(0)->getPhysics();

    BOOST_CHECK_EQUAL(physicsObject.simType, ModelLink::Physics::eStatic);
    BOOST_CHECK_CLOSE(physicsObject.massKg, 0.0, 0.0001);
    BOOST_CHECK_EQUAL(physicsObject.comLocation, ModelLink::Physics::eVisuBBoxCenter);

    ManipulationObjectPtr savedObject = ObjectIO::createManipulationObjectFromString(manipulatioObject->toXML());
    physicsObject = savedObject->getLinks().at(0)->getPhysics();

    BOOST_CHECK_EQUAL(physicsObject.simType, ModelLink::Physics::eStatic);
    BOOST_CHECK_CLOSE(physicsObject.massKg, 0.0, 0.0001);
    BOOST_CHECK_EQUAL(physicsObject.comLocation, ModelLink::Physics::eVisuBBoxCenter);
}
*/


BOOST_AUTO_TEST_SUITE_END()
