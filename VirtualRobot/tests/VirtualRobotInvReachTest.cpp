/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotInvReachTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/InverseReachability.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/MathTools.h>
#include <string>

#define SMALL_VALUE 1e-5
#define SMALL_VALUE2 1e-2

using std::cout;
using std::endl;
BOOST_AUTO_TEST_SUITE(InvReach)


bool isInList(Eigen::Matrix4f& p, std::vector<Eigen::Matrix4f> &list, float precTrans = 10.0f, float precRot=0.5f)
{
    float tr,ro;
    for (auto l : list)
    {
        VirtualRobot::MathTools::getPoseDiff(p,l,tr,ro);
        if (tr<=precTrans && ro<=precRot)
            return true;
    }
    return false;
}

bool hasEntry(VirtualRobot::ReachabilityPtr reach, Eigen::Matrix4f &p, float discrTr, float discrRot, unsigned int neigbors = 1)
{
    Eigen::Matrix4f t = p;
    Eigen::Matrix4f t2;
    if (reach->getEntry(t)>0)
        return true;

    float x[6];
    float x2[6];
    reach->matrix2Vector(p,x);

    for (int a=-neigbors;a<=neigbors;a++)
        for (int b=-neigbors;b<=neigbors;b++)
            for (int c=-neigbors;c<=neigbors;c++)
            {
                for (int d=-neigbors;d<=neigbors;d++)
                    for (int e=-neigbors;e<=neigbors;e++)
                        for (int f=-neigbors;f<=neigbors;f++)
                        {
                            x2[0] = x[0] + discrTr * a;
                            x2[1] = x[1] + discrTr * b;
                            x2[2] = x[2] + discrTr * c;
                            x2[3] = x[3] + discrRot * d;
                            x2[4] = x[4] + discrRot * e;
                            x2[5] = x[5] + discrRot * f;

                            reach->vector2Matrix(x,t);
                            if (reach->getEntry(t)>0)
                                return true;
                        }
            }

    return false;
}

BOOST_AUTO_TEST_CASE(testInvReachCreate)
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
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    std::vector<std::string> rnsNames;
    rnsNames.push_back("joint1");
    VirtualRobot::RobotNodeSetPtr rns = VirtualRobot::RobotNodeSet::createRobotNodeSet(rob, "rns", rnsNames, "", "tcp", true);
    BOOST_REQUIRE(rns);
    BOOST_REQUIRE(rob->hasRobotNodeSet("rns"));

    VirtualRobot::RobotNodePtr joint1 = rob->getRobotNode("joint1");
    BOOST_REQUIRE(joint1);
    VirtualRobot::RobotNodePtr tcp = rob->getRobotNode("tcp");
    BOOST_REQUIRE(tcp);
    VirtualRobot::RobotNodePtr rootNode = rob->getRobotNode("root");
    BOOST_REQUIRE(rootNode);

    // CHECK ROBOT WORKSPACE
    Eigen::Matrix4f tcpPose;
    Eigen::Matrix4f gp;
    float x[6];

    tcpPose = tcp->getGlobalPose();
    VirtualRobot::MathTools::eigen4f2rpy(tcpPose,x);
    BOOST_CHECK_SMALL(fabs(x[0]- 100.0f), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[1] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[2] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[3] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[4] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[5] - 0.0f ), SMALL_VALUE);

    joint1->setJointValue(M_PI/2);
    tcpPose = tcp->getGlobalPose();
    VirtualRobot::MathTools::eigen4f2rpy(tcpPose,x);
    BOOST_CHECK_SMALL(fabs(x[0] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[1] - 100.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[2] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[3] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[4] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[5] - M_PI/2.0f ), SMALL_VALUE);

    joint1->setJointValue(0);
    gp = Eigen::Matrix4f::Identity();
    gp(0,3) = 200.0f;
    gp(1,3) = 500.0f;
    rob->setGlobalPose(gp);
    tcpPose = tcp->getGlobalPose();
    VirtualRobot::MathTools::eigen4f2rpy(tcpPose,x);
    BOOST_CHECK_SMALL(fabs(x[0] - 300.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[1] - 500.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[2] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[3] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[4] - 0.0f ), SMALL_VALUE);
    BOOST_CHECK_SMALL(fabs(x[5] - 0.0f ), SMALL_VALUE);


    // reset rob
    gp = Eigen::Matrix4f::Identity();
    rob->setGlobalPose(gp);
    joint1->setJointValue(0);


    // CREATE REACHABILITY DATA
    static float discrTr = 10.0f;
    static float discrRot = 0.5f;
    float minBounds[6] = {0,0,0,0,0,0};
    float maxBounds[6] = {100.0f,100.0f,100.0f,2*M_PI,2*M_PI,2*M_PI};
    //VirtualRobot::ManipulabilityPtr reach(new VirtualRobot::Manipulability(rob));
    VirtualRobot::ReachabilityPtr reach(new VirtualRobot::Reachability(rob));
    BOOST_REQUIRE(reach);

    reach->initialize(rns, discrTr, discrRot, minBounds, maxBounds, VirtualRobot::SceneObjectSetPtr(), VirtualRobot::SceneObjectSetPtr(), rootNode, tcp);

    float jv = M_PI/4;
    joint1->setJointValue(jv);
    reach->addCurrentTCPPose();

    // this must be enough to fill the reach space
    //reach->addRandomTCPPoses(5000);
    BOOST_REQUIRE(reach);

    std::vector<Eigen::Matrix4f> poses;
    std::vector<Eigen::Matrix4f> posesInv;
    std::vector<Eigen::Matrix4f> posesInvInv;

    float t[6];
    unsigned int v[6];
    //for (float jv = 0.01f; jv<M_PI/2; jv += 0.01f)
    {
        joint1->setJointValue(jv);
        gp = tcp->getGlobalPose();
        reach->matrix2Vector(gp,t);
        cout << "pose:" << t[0] << "," << t[1] << "," << t[2] << "," << t[3] << "," << t[4] << "," << t[5] << endl;
        reach->getVoxelFromPose(gp,v);
        cout << "voxel:" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << endl;
        unsigned char entry = reach->getEntry(gp);
        BOOST_REQUIRE(entry>0);
        if (!isInList(gp, poses))
            poses.push_back(gp);
    }
    std::cout << "nr poses:" << poses.size() << std::endl;

    VirtualRobot::InverseReachabilityPtr invReach(new VirtualRobot::InverseReachability(reach));
    BOOST_REQUIRE(invReach);

    VirtualRobot::WorkspaceDataPtr invData = invReach->getData();


    unsigned int numVoxels[6];
    for (int i=0;i<6; i++)
        numVoxels[i] = invData->getSize(i);

    int countHit = 0;
    int countMiss = 0 ;
    for (v[0] = 0; v[0] < (unsigned int)numVoxels[0]; v[0]++)
        for (v[1] = 0; v[1] < (unsigned int)numVoxels[1]; v[1]++)
            for (v[2] = 0; v[2] < (unsigned int)numVoxels[2]; v[2]++)
                for (v[3] = 0; v[3] < (unsigned int)numVoxels[3]; v[3]++)
                    for (v[4] = 0; v[4] < (unsigned int)numVoxels[4]; v[4]++)
                        for (v[5] = 0; v[5] < (unsigned int)numVoxels[5]; v[5]++)
                        {

                            if (invData->get(v) > 0)
                            {
                                cout << "voxel:" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << endl;

                                Eigen::Matrix4f gp = (invReach->getPoseFromVoxel(v));
                                cout << "gp (inv):" << endl << gp << endl;
                                invReach->matrix2Vector(gp,t);
                                cout << "pose(inv):" << t[0] << "," << t[1] << "," << t[2] << "," << t[3] << "," << t[4] << "," << t[5] << endl;
                                if (!isInList(gp, posesInv))
                                    posesInv.push_back(gp);

                                Eigen::Matrix4f p = gp.inverse();
                                if (!isInList(p, posesInvInv))
                                    posesInvInv.push_back(p);
                                cout << "gp(inv).inv:" << endl << p << endl;
                                invReach->matrix2Vector(p,t);
                                cout << "pose(inv).inv:" << t[0] << "," << t[1] << "," << t[2] << "," << t[3] << "," << t[4] << "," << t[5] << endl;
                                if (hasEntry(reach, p, discrTr, discrRot))
                                    countHit++;
                                else
                                    countMiss++;
                            }
                        }

    std::cout << "countHit:" << countHit << ", countMiss:" << countMiss << std::endl;

    for (auto p:poses)
    {
        std::cout << "p : " << p.block(0,3,3,1).transpose() << std::endl;
    }
    for (auto p:posesInv)
    {
        std::cout << "pI: " << p.block(0,3,3,1).transpose() << std::endl;
    }
    for (auto p:posesInvInv)
    {
        float XII[6];
        invReach->matrix2Vector(p,XII);
        std::cout << "pII: ";
        for (int u=0;u<6;u++)
            std::cout << XII[u] << ",";
         std::cout << std::endl;
    }
 }

BOOST_AUTO_TEST_SUITE_END()
