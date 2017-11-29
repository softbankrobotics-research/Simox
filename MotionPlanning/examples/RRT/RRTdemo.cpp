
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/Tools/RuntimeEnvironment.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "MotionPlanning/MotionPlanning.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/BiRrt.h"
#include "MotionPlanning/Visualization/RrtWorkspaceVisualization.h"

#include <VirtualRobot/Import/SimoxXMLFactory.h>

#include "../../../Gui/ViewerInterface.h"
#include "../../../Gui/ViewerFactory.h"

#include "string"
#include <iostream>

#include <QWidget>

using std::cout;
using std::endl;
using namespace VirtualRobot;
using namespace MotionPlanning;

#define USE_BIRRT

bool useColModel = false;
QWidget* win;

int show(std::vector<VisualizationSetPtr> &visus)
{
    if (win == nullptr)
    {
        printf("Could not create window.\n");
        exit(-3);
    }

    SimoxGui::ViewerInterfacePtr viewer;
    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(win);

    // set the visu
    int i = 0;
    for (auto &n: visus)
    {
        std::stringstream vn;
        vn << "rrt-" << i;
        viewer->addVisualization("rrt", n);
        i++;
    }

    // show everything
    viewer->viewAll();

    win->show();
    win->raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}

int startRRTVisualization()
{

    // create robot
    std::string filename("robots/SimoxXML/examples/RrtDemo/Joint3.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    cout << "Loading 3DOF robot from " << filename << endl;
    RobotPtr robot = SimoxXMLFactory::loadRobotSimoxXML(filename);

    if (!robot)
    {
        return 1;
    }


    float sampling_extend_stepsize = 0.04f;

    // create environment
    float posX = 0.0f, posY = -400.0f, posZ = 400.0f, sizeX = 100.0f, sizeY = 300.0f, sizeZ = 1000.0f;
    ObstaclePtr o = Obstacle::createBox(sizeX, sizeY, sizeZ);

    Eigen::Affine3f tmpT(Eigen::Translation3f(posX, posY, posZ));
    o->setGlobalPose(tmpT.matrix());

    // setup collision detection
    std::string colModelName("CollisionModel");
    LinkSetPtr cms = robot->getLinkSet(colModelName);
    CDManagerPtr cdm(new CDManager());
    cdm->addCollisionModel(cms);
    cdm->addCollisionModel(o->getLinkSet());

    float planningTime = 0;
    int failed = 0;
    int loops = 1;
    bool ok;
    CSpaceSampledPtr cspace;
    std::string planningJoints("AllJoints");
    JointSetPtr planningNodes = robot->getJointSet(planningJoints);
#ifdef USE_BIRRT
    BiRrtPtr rrt;
#else
    RrtPtr rrt;
#endif
    Eigen::VectorXf start(3);
    start(0) = (float)M_PI / 4.0f;
    start(1) = (float)M_PI / 4.0f * 1.5f;
    start(2) = (float) - M_PI / 4.0f;

    Eigen::VectorXf goal(3);
    goal(0) = (float) - M_PI / 4.0f;
    goal(1) = (float)M_PI / 4.0f * 1.5f;
    goal(2) = (float) - M_PI / 4.0f;

    for (int i = 0; i < loops; i++)
    {
        // setup C-Space

        cspace.reset(new CSpaceSampled(robot, cdm, planningNodes));
        cspace->setSamplingSize(sampling_extend_stepsize);
        cspace->setSamplingSizeDCD(sampling_extend_stepsize);
        // setup planner


#ifdef USE_BIRRT
        rrt.reset(new BiRrt(cspace));
#else
        rrt.reset(new Rrt(cspace));
#endif
        rrt->setStart(start);
        rrt->setGoal(goal);

        clock_t startT = clock();
        ok = rrt->plan(true);
        clock_t endT = clock();

        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        planningTime += diffClock;

        if (!ok)
        {
            failed++;
        }
    }

    planningTime /= (float)loops;
    cout << "Avg planning time: " << planningTime << endl;
    cout << "failed:" << failed << endl;

    if (!ok)
    {
        cout << "planning failed..." << endl;
        return 1;
    }

    CSpacePathPtr solution = rrt->getSolution();
    CSpaceTreePtr tree = rrt->getTree();

    planningNodes->setJointValues(start);

    // display robot
    ModelLink::VisualizationType colModel = ModelLink::VisualizationType::Full;

    VisualizationSetPtr visualisationRobot = robot->getVisualization(colModel);


    // display obstacle
    VisualizationSetPtr visualisationObstacle = o->getVisualization(colModel);

    // show rrt visu

    RrtWorkspaceVisualizationPtr w(new RrtWorkspaceVisualization(robot, cspace, "EndPoint"));

    w->addTree(tree);
#ifdef USE_BIRRT
    CSpaceTreePtr tree2 = rrt->getTree2();
    w->addTree(tree2);
#endif
    w->addCSpacePath(solution);
    w->addConfiguration(start, RrtWorkspaceVisualization::eGreen, 3.0f);
    w->addConfiguration(goal, RrtWorkspaceVisualization::eGreen, 3.0f);

    VisualizationSetPtr visuRrt = w->getVisualization();

    std::vector<VisualizationSetPtr> visus;
    visus.push_back(visualisationRobot);
    visus.push_back(visualisationObstacle);
    visus.push_back(visuRrt);

    return show(visus);
}

int main(int argc, char** argv)
{
    VirtualRobot::init(argc, argv, "RRT-Demo");
    win = new QWidget();
    win->resize(640,480);
    cout << " --- START --- " << endl;

    int status = 1;

    try
    {
        status = startRRTVisualization();
    }
    catch (VirtualRobot::VirtualRobotException v)
    {
        std::cout << "VirtualRobot Exception: " << v.what() << std::endl ;
    }
    catch (std::exception e)
    {
        std::cout << "Exception: " << e.what() << std::endl ;
    }
    catch (...)
    {
        ;
    }

    cout << " --- END --- " << endl;

    return status;
}
