/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotRobotTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/Model/Nodes/Attachments/PositionSensor.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <string>


BOOST_AUTO_TEST_SUITE(ModelFactory)

BOOST_AUTO_TEST_CASE(testModelFactoryEmptyXML)
{
    const std::string robotString = "";
    BOOST_REQUIRE_THROW((VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString)), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryUnclosedRobotTag)
{
    const std::string robotString = "<Robot>";
    VirtualRobot::RobotPtr robot;
    BOOST_REQUIRE_THROW(robot = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryOnlyClosedRobotTag)
{
    const std::string robotString = "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryEmptyRobotTag)
{
    const std::string robotString = "<Robot></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryEmptyTypeString)
{
    const std::string robotString = "<Robot Type=''></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryNotExistentType)
{
    const std::string robotString = "<Robot Type='XYZ'></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotRobotMacro)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node = "Joint1";
    VirtualRobot::RobotNodePtr r1 = rob->getNode(node);
    BOOST_REQUIRE(r1);
}

BOOST_AUTO_TEST_CASE(testModelFactoryEmptyRootNodeString)
{
    const std::string robotString = "<Robot RootNode=''></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testModelFactoryNotExistentRootNode)
{
    const std::string robotString = "<Robot RootNode='JointX'></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotValidEndeffector)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"
        " <RobotNode name='Joint3'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint4'/>"
        " </RobotNode>"
        " <RobotNode name='Joint4'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "   <Node name='Joint2'/>"
        "  </Static>"
        "  <Actor name='actor1'>"
        "   <Node name='Joint3'/>"
        "   <Node name='Joint4'/>"
        "  </Actor>"
        " </Endeffector>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotInvariantTagPosition)
{
    // test if references to nodes are resolved correctly if the
    // nodes are defined after they are referenced
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint3'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint4'/>"
        " </RobotNode>"
        " <RobotNode name='Joint4'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "   <Node name='Joint2'/>"
        "  </Static>"
        "  <Actor name='actor1'>"
        "   <Node name='Joint3'/>"
        "   <Node name='Joint4'/>"
        "  </Actor>"
        " </Endeffector>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWrongChildTag)
{
    int argc = 0;
    VirtualRobot::VisualizationFactory::getInstance()->init(argc,nullptr, "VirtualRobotRobotTest");

    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "   <CoordinateAxis enable='true' scaling='1' text='Axis1'/>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <XYZ>"
        "  </XYZ>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "  </Static>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), std::exception);//VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWithoutNameTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector base='Joint1' tcp='Joint1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenodeTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector name ='e1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenode)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector name ='e1' base='Joint1' tcp='Joint1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotPhysicsTag)
{
    const std::string wrongRobotString =
        "<Robot Type='DemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Physics>"
        "   <Mass value='100' units='kg'/>"
        "   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
        "   <InertiaMatrix unitsWeight='ton' unitsLength='mm'>"
        "     <row1 c1='1' c2='2' c3='3'/>"
        "     <row2 c1='4' c2='5' c3='6'/>"
        "     <row3 c1='7' c2='8' c3='9'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Transform>"
        "    <Translation x='1' y='0' z='0' unitsLength='m'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    // physics in joints is not allowed!
    BOOST_REQUIRE_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(wrongRobotString), VirtualRobot::VirtualRobotException);

    const std::string robotString2 =
        "<Robot Type='DemoRobotType' RootNode='Body1'>"
        " <RobotNode name='Body1'>"
        "  <Physics>"
        "   <Mass value='100' units='kg'/>"
        "   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
        "   <InertiaMatrix unitsWeight='ton' unitsLength='mm'>"
        "     <row1 c1='1' c2='2' c3='3'/>"
        "     <row2 c1='4' c2='5' c3='6'/>"
        "     <row3 c1='7' c2='8' c3='9'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Transform>"
        "    <Translation x='1' y='0' z='0' unitsLength='m'/>"
        "  </Transform>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        " </RobotNode>"
        "</Robot>";

    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString2));
    BOOST_REQUIRE(rob);
    VirtualRobot::ModelJointPtr joint = rob->getJoint("Joint2");
    BOOST_REQUIRE(joint);
    VirtualRobot::ModelLinkPtr link = rob->getLink("Body1");
    BOOST_REQUIRE(link);
    float mass = link->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    float vel = joint->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    float acc = joint->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    float to = joint->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    Eigen::Vector3f com = link->getCoMLocal();
    bool comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    Eigen::Matrix3f inertia = link->getInertiaMatrix();
    Eigen::Matrix3f expectedMat;
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    bool inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);
    Eigen::Matrix4f m = link->getNodeTransformation();
    BOOST_CHECK_EQUAL(m(0, 3), 1000.0f);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotDependendNodes)
{
    //        "    <DH a='1' d='0' theta='0' alpha='-90' units='degree' unitsLength='m'/>"
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Transform>"
        "    <Translation x='1' y='0' z='0' unitsLength='m'/>"
        "    <rollpitchyaw yaw='-90' units='degree'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='180'/>"
        "    <PropagateJointValue factor='0.5' name='Joint2'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "   <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='90'/>"
        "   </Joint>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    VirtualRobot::ModelJointPtr r1 = rob->getJoint(node1);
    BOOST_REQUIRE(r1);
    VirtualRobot::ModelJointPtr r2 = rob->getJoint(node2);
    BOOST_REQUIRE(r2);
    float j1, j2;
    r1->setJointValue(0.2f);
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_EQUAL(j1, 0.2f);
    BOOST_CHECK_EQUAL(j2, 0.1f);
    r1->setJointValue(float(M_PI));
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_CLOSE(j1, float(M_PI), 0.1f);
    BOOST_CHECK_CLOSE(j2, float(M_PI / 2.0), 0.1f);

    // disable propagate feature
    r1->propagateJointValue("Joint2", 0.0f);
    r2->setJointValue(0.5f);
    r1->setJointValue(0.2f);
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_CLOSE(j1, 0.2f, 0.1f);
    BOOST_CHECK_CLOSE(j2, 0.5f, 0.1f);

}


BOOST_AUTO_TEST_CASE(testVirtualRobotToXML)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Body1'>"
        " <RobotNode name='Body1'>"
        "  <Physics>"
        "   <Mass value='100' units='kg'/>"
        "   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
        "   <InertiaMatrix unitsWeight='ton' unitsLength='mm'>"
        "     <row1 c1='1' c2='2' c3='3'/>"
        "     <row2 c1='4' c2='5' c3='6'/>"
        "     <row3 c1='7' c2='8' c3='9'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='180'/>"
        "    <PropagateJointValue factor='0.5' name='Joint3'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        "   <Sensor type='position' name='sensor'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
         "  </Sensor>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"
        " <RobotNode name='Joint3'>"
        "  <Transform>"
        "    <Translation x='100' y='50' z='0'/>"
        "  </Transform>"
        "   <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='90'/>"
        "   </Joint>"
        "   <Sensor type='position' name='sensor2'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
         "  </Sensor>"
        " </RobotNode>"
        "</Robot>";

    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Body1";
    const std::string node2 = "Joint2";
    const std::string node3 = "Joint3";
    VirtualRobot::ModelLinkPtr body1 = rob->getLink(node1);
    BOOST_REQUIRE(body1);
    VirtualRobot::ModelJointPtr joint1 = rob->getJoint(node2);
    VirtualRobot::ModelJointPtr joint2 = rob->getJoint(node3);
    BOOST_REQUIRE(joint1);
    BOOST_REQUIRE(joint2);

    // check physics
    float mass = body1->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    float vel = joint1->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    float acc = joint1->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    float to = joint1->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    Eigen::Vector3f com = body1->getCoMLocal();
    bool comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    Eigen::Matrix3f inertia = body1->getInertiaMatrix();
    Eigen::Matrix3f expectedMat;
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    bool inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);


    // check sensor
    BOOST_REQUIRE(rob->hasSensor("sensor2"));
    VirtualRobot::PositionSensorPtr ps = std::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rob->getSensor("sensor2"));
    BOOST_REQUIRE(ps);
    Eigen::Matrix4f p = ps->getGlobalPose();
    Eigen::Matrix4f p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 200.0f, 100.0f, 0;
    BOOST_REQUIRE(p.isApprox(p2));


    // todo
    // create xml robot
    /*
    std::string robXML;
    BOOST_REQUIRE_NO_THROW(robXML = rob->toXML());
    BOOST_REQUIRE(!robXML.empty());

    VirtualRobot::RobotPtr rob2;
    BOOST_REQUIRE_NO_THROW(rob2 = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robXML));
    BOOST_REQUIRE(rob2);

	VirtualRobot::ModelLinkPtr rn1 = rob2->getLink(node1);
    BOOST_REQUIRE(rn1);
	VirtualRobot::ModelLinkPtr rn2 = rob2->getLink(node2);
    BOOST_REQUIRE(rn2);

    // check physics
    mass = rn1->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    vel = rn1->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    acc = rn1->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    to = rn1->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    com = rn1->getCoMLocal();
    comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    inertia = rn1->getInertiaMatrix();
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);*/


    // check sensor
    /*BOOST_REQUIRE(rn2->hasSensor("sensor2"));
    ps = std::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rn2->getSensor("sensor2"));
    BOOST_REQUIRE(ps);
    p = ps->getGlobalPose();
    p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 200.0f, 100.0f, 0;
    BOOST_REQUIRE(p.isApprox(p2));*/
}

BOOST_AUTO_TEST_SUITE_END()
