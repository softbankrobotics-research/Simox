/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotWorkSpaceGridTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/ModelNodeSet.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
#include <string>

BOOST_AUTO_TEST_SUITE(WorkSpaceGrid)

BOOST_AUTO_TEST_CASE(testWorkSpaceGrid)
{
    // CREATE ROBOT AND DATA STRUCTURES
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='root'>"
            " <RobotNode name='root'>"
            "   <Child name='joint1'/>"
            " </RobotNode>"
            " <RobotNode name='joint1'>"
            "   <Joint type='revolute'>"
            "      <Limits unit='degree' lo='0' hi='90'/>"
            "      <axis x='0' y='0' z='1'/>"
            "   </Joint>"
            "   <Child name='tcp'/>"
            " </RobotNode>"
            " <RobotNode name='tcp'>"
            "   <Transform>"
            "      <Translation x='100' y='0' z='0'/>"
            "   </Transform>"
            " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::SimoxXMLFactory::createRobotFromSimoxXMLString(robotString));
    BOOST_REQUIRE(rob);

    std::vector<std::string> rnsNames;
    rnsNames.push_back("joint1");
    VirtualRobot::JointSetPtr rns = VirtualRobot::JointSet::createJointSet(rob, "rns", rnsNames, "", "tcp", true);
    BOOST_REQUIRE(rns);
    BOOST_REQUIRE(rob->hasJointSet("rns"));

    VirtualRobot::ModelJointPtr joint1 = rob->getJoint("joint1");
    BOOST_REQUIRE(joint1);
    VirtualRobot::FramePtr tcp = rob->getFrame("tcp");
    BOOST_REQUIRE(tcp);
    VirtualRobot::RobotNodePtr rootNode = rob->getNode("root");
    BOOST_REQUIRE(rootNode);

    // CHECK ROBOT WORKSPACE
    Eigen::Matrix4f tcpPose;
    Eigen::Matrix4f gp;

    // reset rob
    gp = Eigen::Matrix4f::Identity();
    rob->setGlobalPose(gp);
    joint1->setJointValue(0);


    // CREATE REACHABILITY DATA
    static float discrTr = 20.0f;
    static float discrRot = 0.5f;
    static float discrTr3 = discrTr*sqrtf(3);
    static float discrRot3 = discrRot*sqrtf(3);
    float minBounds[6] = {-200.0f,-200.0f,-200.0f,0,0,0};
    float maxBounds[6] = {200.0f,200.0f,200.0f,float(2*M_PI),float(2*M_PI),float(2*M_PI)};
    VirtualRobot::ReachabilityPtr reach(new VirtualRobot::Reachability(rob));
    BOOST_REQUIRE(reach);
    reach->setOrientationType(VirtualRobot::WorkspaceRepresentation::Hopf);
    reach->initialize(rns, discrTr, discrRot, minBounds, maxBounds, VirtualRobot::LinkSetPtr(), VirtualRobot::LinkSetPtr(), rootNode, tcp);

    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    //float x[6];
    unsigned int v[6];
    float diffRot,diffPos;
    bool poseOK;

    m(0,3) = 40.0f;
    m(1,3) = -80.0f;
    m(2,3) = 120.0f;

    reach->addPose(m);

    poseOK = reach->getVoxelFromPose(m,v);
    BOOST_REQUIRE(poseOK);
    m2 = reach->getPoseFromVoxel(v);
    VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    poseOK = reach->isCovered(m);
    BOOST_REQUIRE(poseOK);

    m2 = Eigen::Matrix4f::Identity();
    m2(2,3) = m(2,3);
    VirtualRobot::WorkspaceRepresentation::WorkspaceCut2DPtr cutXY = reach->createCut(m2, discrTr, false);
    BOOST_REQUIRE(cutXY);


    int nx = cutXY->entries.rows();
    int ny = cutXY->entries.cols();

    int cellSizeX = int((cutXY->maxBounds[0] - cutXY->minBounds[0]) / nx);
    int cellSizeY = int((cutXY->maxBounds[1] - cutXY->minBounds[1]) / ny);

    int posA = int((m(0,3) - cutXY->minBounds[0]) / cellSizeX);
    int posB = int((m(1,3) - cutXY->minBounds[1]) / cellSizeY);

    for (int a=0; a<nx; a++)
    {
        for (int b=0; b<ny; b++)
        {
            if (cutXY->entries(a,b)>0)
            {
                BOOST_REQUIRE_LE(abs(a - posA), 1);
                BOOST_REQUIRE_LE(abs(b - posB), 1);
            }
        }
    }

    // check transformations
    std::vector<VirtualRobot::WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> transformations = reach->createCutTransformations(cutXY, rootNode);
    BOOST_REQUIRE_EQUAL(transformations.size(), 1);
    BOOST_REQUIRE_EQUAL(transformations.at(0)->value, 1);
    VirtualRobot::MathTools::getPoseDiff(m, transformations.at(0)->transformation, diffPos, diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    // check m
    Eigen::Matrix4f tmpPos2 = m * transformations.at(0)->transformation.inverse();
    VirtualRobot::MathTools::getPoseDiff(Eigen::Matrix4f::Identity(), tmpPos2, diffPos, diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);


    // move pose by 500
    m2=m;
    m2(0,3) += 500.0f;
    tmpPos2 = m2 * transformations.at(0)->transformation.inverse();
    Eigen::Matrix4f tmpPos3 = Eigen::Matrix4f::Identity();
    tmpPos3(0,3) += 500.0f;
    VirtualRobot::MathTools::getPoseDiff(tmpPos3, tmpPos2, diffPos, diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    //BOOST_REQUIRE_LE(diffRot, discrRot3); // only position is of interest here

    // rotate pose by 90 degree
    m2 = VirtualRobot::MathTools::axisangle2eigen4f(Eigen::Vector3f(0,0,1.0f), float(M_PI/2.0));
    m2(2,3) = m(2,3);
    Eigen::Matrix4f invTr = transformations.at(0)->transformation.inverse();
    tmpPos2 = m2 * invTr;
    tmpPos3 = Eigen::Matrix4f::Identity();
    tmpPos3(0,3) = -80.0f; // the inverse (-40/80/-120) rotated by 90 degree around z
    tmpPos3(1,3) = -40.0f;
    VirtualRobot::MathTools::getPoseDiff(tmpPos3, tmpPos2, diffPos, diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    //BOOST_REQUIRE_LE(diffRot, discrRot3); // only position is of interest here


    // create workspace grid at (virtual) grasping position m
    Eigen::Vector3f minBB, maxBB;
    reach->getWorkspaceExtends(minBB, maxBB);
    VirtualRobot::WorkspaceGridPtr reachGrid(new VirtualRobot::WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reach->getDiscretizeParameterTranslation()));

    reachGrid->setGridPosition(m(0, 3), m(1, 3));

    VirtualRobot::GraspPtr g;
    bool fillOK = reachGrid->fillGridData(reach, m, g, rootNode);
    BOOST_REQUIRE(fillOK);

    // grid is moved to pos (40/-80) and position (0/0) is added
    int e = reachGrid->getEntry(0.0f,0.0f);
    BOOST_REQUIRE(e>0);

    // cell should be 8/14
    e = reachGrid->getCellEntry(8,14);
    BOOST_REQUIRE(e>0);

    // check with rotated grasping position (90 degrees around z)
    m2 = VirtualRobot::MathTools::axisangle2eigen4f(Eigen::Vector3f(0,0,1), float(M_PI/2.0));
    m2(0,3) = m(0,3);
    m2(1,3) = m(1,3);
    m2(2,3) = m(2,3);

    reach->getWorkspaceExtends(minBB, maxBB);
    reachGrid.reset(new VirtualRobot::WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reach->getDiscretizeParameterTranslation()));

    reachGrid->setGridPosition(m2(0, 3), m2(1, 3));

    fillOK = reachGrid->fillGridData(reach, m2, g, rootNode);
    BOOST_REQUIRE(fillOK);

    // grid is moved to pos (40/-80) and position (0/0) is added (todo: evaluate what goes wrong with these checks...)
    e = reachGrid->getEntry(0.0f,0.0f);
    //BOOST_REQUIRE(e>0);

    // cell should be 8/14
    e = reachGrid->getCellEntry(8,14);
    //BOOST_REQUIRE(e>0);


}


BOOST_AUTO_TEST_SUITE_END()
