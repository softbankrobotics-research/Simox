/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotWorkSpaceTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/Model/ModelNodeSet.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Import/SimoxXMLFactory.h>
#include <string>

BOOST_AUTO_TEST_SUITE(WorkSpace)

BOOST_AUTO_TEST_CASE(testWorkSpaceEuler)
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
    VirtualRobot::WorkspaceRepresentationPtr ws;
    BOOST_REQUIRE_NO_THROW(ws.reset(new VirtualRobot::WorkspaceRepresentation(rob)));
    ws->setOrientationType(VirtualRobot::WorkspaceRepresentation::Hopf);
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    float x[6];

    // identity, matrix -> vector
    ws->matrix2Vector(m, x);

    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[3], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[4], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[5], 0.0f, 1e-6f);

    // identity, vector -> matrix
    for (int i = 0; i < 6; i++)
    {
        x[i] = 0.0f;
    }

    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    Eigen::Vector3f ax;
    float a;
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, 0.0f, 1e-6f);

    // rot x
    m.setIdentity();
    Eigen::Matrix3f m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitX()).matrix();
    m.block(0, 0, 3, 3) = m3;


    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(0), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(1), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(2), 0.0f, 1e-6f);


    // rot y
    m.setIdentity();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitY()).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(1), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(0), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(2), 0.0f, 1e-6f);

    // rot z
    m.setIdentity();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitZ()).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(2), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(1), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(0), 0.0f, 1e-6f);


    // rot x,y
    m.setIdentity();
    ax << 1.0f, 1.0f, 0.0f;
    ax.normalize();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, ax).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(0), 1.0f / sqrt(2.0f), 1e-3f);
    BOOST_CHECK_CLOSE(ax(1), 1.0f / sqrt(2.0f), 1e-3f);
    BOOST_CHECK_SMALL(ax(2), 1e-4f);

}

BOOST_AUTO_TEST_CASE(testWorkSpaceNeighbors)
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
    BOOST_REQUIRE(rob->hasModelNodeSet("rns"));

    VirtualRobot::ModelJointPtr joint1 = rob->getJoint("joint1");
    BOOST_REQUIRE(joint1);
    VirtualRobot::RobotNodePtr tcp = rob->getModelNode("tcp");
    BOOST_REQUIRE(tcp);
    VirtualRobot::RobotNodePtr rootNode = rob->getModelNode("root");
    BOOST_REQUIRE(rootNode);

    // CHECK ROBOT WORKSPACE
    Eigen::Matrix4f tcpPose;
    Eigen::Matrix4f gp;

    // reset rob
    gp = Eigen::Matrix4f::Identity();
    rob->setGlobalPose(gp);
    joint1->setJointValue(0);


    // CREATE REACHABILITY DATA
    static float discrTr = 10.0f;
    static float discrRot = 0.5f;
    static float discrTr3 = float(discrTr*sqrt(3));
    static float discrRot3 = float(discrRot*sqrt(3));
    float minBounds[6] = {0,0,0,0,0,0};
    float maxBounds[6] = {100.0f,100.0f,100.0f,2*M_PI,2*M_PI,2*M_PI};
    VirtualRobot::ReachabilityPtr reach(new VirtualRobot::Reachability(rob));
    BOOST_REQUIRE(reach);
    reach->setOrientationType(VirtualRobot::WorkspaceRepresentation::Hopf);
    reach->initialize(rns, discrTr, discrRot, minBounds, maxBounds, VirtualRobot::LinkSetPtr(), VirtualRobot::LinkSetPtr(), rootNode, tcp);
/*
    float jv = M_PI/4;
    joint1->setJointValue(jv);
    reach->addCurrentTCPPose();
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    float x[6];*/

    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    float x[6];
    unsigned int v[6];
    float diffRot,diffPos;
    bool poseOK;

    // identity, matrix -> voxel -> matrix
    poseOK = reach->getVoxelFromPose(m,v);
    BOOST_REQUIRE(poseOK);
    m2 = reach->getPoseFromVoxel(v);
    VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    // rot x, matrix -> voxel -> matrix
    m.setIdentity();
    Eigen::Matrix3f m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitX()).matrix();
    m.block(0, 0, 3, 3) = m3;
    poseOK = reach->getVoxelFromPose(m,v);
    BOOST_REQUIRE(poseOK);
    m2 = reach->getPoseFromVoxel(v);
    VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    // rot y, matrix -> voxel -> matrix
    m.setIdentity();
    m3 = Eigen::AngleAxisf(-float(M_PI) / 4.0f, Eigen::Vector3f::UnitY()).matrix();
    m.block(0, 0, 3, 3) = m3;
    poseOK = reach->getVoxelFromPose(m,v);
    BOOST_REQUIRE(poseOK);
    m2 = reach->getPoseFromVoxel(v);
    VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    // rot z, matrix -> voxel -> matrix
    m.setIdentity();
    m3 = Eigen::AngleAxisf(float(M_PI) / 2.0f, Eigen::Vector3f::UnitZ()).matrix();
    m.block(0, 0, 3, 3) = m3;
    poseOK = reach->getVoxelFromPose(m,v);
    BOOST_REQUIRE(poseOK);
    m2 = reach->getPoseFromVoxel(v);
    VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
    BOOST_REQUIRE_LE(diffPos, discrTr3);
    BOOST_REQUIRE_LE(diffRot, discrRot3);

    const int NR_TESTS = 1000;
    for (int i=0;i<NR_TESTS;i++)
    {
        Eigen::Vector3f ax = Eigen::Vector3f::Random();
        ax.normalize();
        float ang = rand() % 1000 / 1000.0f * 2.0f*M_PI -M_PI;
        float xa = rand() % 1000 / 1000.0f * 100.0f;
        float ya = rand() % 1000 / 1000.0f * 100.0f;
        float za = rand() % 1000 / 1000.0f * 100.0f;
        m3 = Eigen::AngleAxisf(ang, ax).matrix();
        m.block(0, 0, 3, 3) = m3;
        m(0,3) = xa;
        m(1,3) = ya;
        m(2,3) = za;

        reach->matrix2Vector(m, x);
        reach->vector2Matrix(x, m2);
        VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
        if (diffPos > discrTr || diffRot > discrRot)
        {
            std::cout << "matrix<->vect: Pose diff: tr:" << diffPos << ", rot:" << diffRot << ", p:" << xa << "," << ya << "," << za << ", ax:" << ax.transpose() << ", ang:" << ang << std::endl;
        }
        BOOST_REQUIRE_LE(diffPos, discrTr3);
        BOOST_REQUIRE_LE(diffRot, discrRot3);


        poseOK = reach->getVoxelFromPose(m,v);
        BOOST_REQUIRE(poseOK);
        if (!poseOK)
        {
            std::cout << "pose!ok:" << m << std::endl;
            //reach->getVoxelFromPose(m,v);
            continue;
        }
        m2 = reach->getPoseFromVoxel(v);
        /*float x2[6];
        float x3[6];
        float v2[6];
        float v3[6];
        for (int i=0;i<6;i++)
            v2[i] = (float)v[i];
        for (int i=0;i<6;i++)
            v3[i] = (float)v[i] + 0.5f;
        reach->getPoseFromVoxelLocal(v2,x2);
        reach->getPoseFromVoxelLocal(v3,x3);*/

        VirtualRobot::MathTools::getPoseDiff(m,m2,diffPos,diffRot);
        if (diffPos > discrTr3 || diffRot > discrRot3)
        {
            std::cout << "Pose diff: tr:" << diffPos << ", rot:" << diffRot << ", p:" << xa << "," << ya << "," << za << ", ax:" << ax.transpose() << ", ang:" << ang << std::endl;
            /*std::cout << "vOrig:";
            for (int i=0;i<6;i++)
                std::cout << v[i] << ",";
            std::cout << std::endl;
            reach->getVoxelFromPose(m2,v);
            std::cout << "v2New:";
            for (int i=0;i<6;i++)
                std::cout << v[i] << ",";
            std::cout << std::endl;
            std::cout << "xOrig:";
            for (int i=0;i<6;i++)
                std::cout << x[i] << ",";
            std::cout << std::endl;
            std::cout << "x2New:";
            for (int i=0;i<6;i++)
                std::cout << x2[i] << ",";
            std::cout << std::endl;
            std::cout << "x3+0.5:";
            for (int i=0;i<6;i++)
                std::cout << x3[i] << ",";
            std::cout << std::endl;

            float xT1[6];
            xT1[0] = x[0];
            xT1[1] = x[1];
            xT1[2] = x[2];
            xT1[3] = x2[3];
            xT1[4] = x2[4];
            xT1[5] = x2[5];
            Eigen::Matrix4f mT1;
            reach->vector2Matrix(xT1,mT1);
            VirtualRobot::MathTools::getPoseDiff(m,mT1,diffPos,diffRot);
            std::cout << "T1 Pose diff: tr:" << diffPos << ", rot:" << diffRot << std::endl;*/
         }
        BOOST_REQUIRE_LE(diffPos, discrTr3);
        BOOST_REQUIRE_LE(diffRot, discrRot3);

    }
}


BOOST_AUTO_TEST_SUITE_END()
