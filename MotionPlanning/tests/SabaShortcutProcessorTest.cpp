/**
* @package    MotionPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE MotionPlanning_MotionPlanningShortcutProcessorTest

#include "VirtualRobot/VirtualRobotTest.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/ModelNodeSet.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/CollisionDetection/CollisionModel.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include "../PostProcessing/ShortcutProcessor.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/Import/SimoxXMLFactory.h"

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>


BOOST_AUTO_TEST_SUITE(CSpaceShortcutProcessor)


BOOST_AUTO_TEST_CASE(testShortcutProcessor)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "   <Joint type='revolute'>"
        "     <Limits unit='degree' lo='-180' hi='180'/>"
        "	  <Axis x='1' y='0' z='0'/>"
        "   </Joint>"
        "   <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "   <Joint type='revolute'>"
        "     <Limits unit='degree' lo='-180' hi='180'/>"
        "	  <Axis x='0' y='1' z='0'/>"
        "   </Joint>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        "  <Node name='Joint2'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString);
    BOOST_REQUIRE(rob);
    VirtualRobot::JointSetPtr rns = rob->getJointSet("rns1");
    BOOST_REQUIRE(rns);
    VirtualRobot::CDManagerPtr cdm(new VirtualRobot::CDManager());
    MotionPlanning::CSpaceSampledPtr cspace(new MotionPlanning::CSpaceSampled(rob, cdm, rns));
    BOOST_REQUIRE(cspace);

    Eigen::VectorXf p1(2);
    Eigen::VectorXf p2(2);
    Eigen::VectorXf p3(2);
    Eigen::VectorXf p4(2);

    p1 << -1, 0;
    p2 << 0, 1;
    p3 << 1, 1;
    p4 << 1, 0;

    MotionPlanning::CSpacePathPtr path(new MotionPlanning::CSpacePath(cspace, "test_path"));
    BOOST_REQUIRE(path);

    path->addPoint(p1);
    path->addPoint(p2);
    path->addPoint(p3);
    path->addPoint(p4);

    MotionPlanning::ShortcutProcessorPtr sc(new MotionPlanning::ShortcutProcessor(path, cspace));
    BOOST_REQUIRE(sc);
    int loops = 100;
    int startI, endI;
    bool res;

    for (int i = 0; i < loops; i++)
    {
        res = sc->selectCandidatesRandom(startI, endI);
        BOOST_CHECK_EQUAL(res, true);
        bool startOK = startI == 0 || startI == 1;
        BOOST_CHECK_EQUAL(startOK, true);
        bool endOK = endI == 2 || endI == 3;
        BOOST_CHECK_EQUAL(endOK, true);
        bool distOK = (endI - startI) >= 2;
        BOOST_CHECK_EQUAL(distOK, true);
    }

    res = sc->validShortcut(0, 2);
    BOOST_CHECK_EQUAL(res, true);
    res = sc->validShortcut(0, 3);
    BOOST_CHECK_EQUAL(res, true);
    res = sc->validShortcut(1, 3);
    BOOST_CHECK_EQUAL(res, true);

    float l = path->getLength();
    MotionPlanning::CSpacePathPtr c1 = path->clone();
    MotionPlanning::CSpacePathPtr c2 = path->clone();
    MotionPlanning::CSpacePathPtr c3 = path->clone();
    MotionPlanning::ShortcutProcessorPtr sc1(new MotionPlanning::ShortcutProcessor(c1, cspace));
    MotionPlanning::ShortcutProcessorPtr sc2(new MotionPlanning::ShortcutProcessor(c2, cspace));
    MotionPlanning::ShortcutProcessorPtr sc3(new MotionPlanning::ShortcutProcessor(c3, cspace));

    sc1->doShortcut(0, 2);
    MotionPlanning::CSpacePathPtr o1 = sc1->getOptimizedPath();
    BOOST_REQUIRE(o1);
    float l1 = o1->getLength();
    BOOST_CHECK_LE(l1, l);

    sc2->doShortcut(0, 3);
    MotionPlanning::CSpacePathPtr o2 = sc2->getOptimizedPath();
    BOOST_REQUIRE(o2);
    float l2 = o2->getLength();
    BOOST_CHECK_LE(l2, l);

    sc3->doShortcut(1, 3);
    MotionPlanning::CSpacePathPtr o3 = sc3->getOptimizedPath();
    BOOST_REQUIRE(o3);
    float l3 = o3->getLength();
    BOOST_CHECK_LE(l3, l);


}

BOOST_AUTO_TEST_SUITE_END()
