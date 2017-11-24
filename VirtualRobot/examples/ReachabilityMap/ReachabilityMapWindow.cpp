
#include "ReachabilityMapWindow.h"
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <Gui/ViewerFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>

#include <QFileDialog>

#include <Eigen/Geometry>

#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

// load static factories from SimoxGui-lib.
// TODO this workaround is actually something we should avoid
#ifdef Simox_USE_COIN_VISUALIZATION
    #include <Gui/Coin/CoinViewerFactory.h>
    SimoxGui::CoinViewerFactory f;
#endif

//#define ENDLESS

ReachabilityMapWindow::ReachabilityMapWindow(std::string& sRobotFile, std::string& reachFile, std::string& objFile, std::string& eef)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << endl;

    robotFile = sRobotFile;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFile);

    robotVisuLayer = "robot-layer";
    reachVisuLayer = "reach-layer";
    reachMapVisuLayer = "reachMap-layer";
    allGraspsVisuLayer = "allGrasps-layer";
    graspVisuLayer = "grasp-layer";
    objectVisuLayer = "object-layer";

    setupUI();

    loadRobot();

    if (!reachFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
        {
            loadReachFile(reachFile);
        }
    }

    if (!objFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(objFile))
        {
            loadObjectFile(objFile);
        }
    }

    setupEnvironment();
    updateVisu();

    if (!eef.empty())
    {
        selectEEF(eef);
    }

    viewer->viewAll();
}


ReachabilityMapWindow::~ReachabilityMapWindow()
{
}


void ReachabilityMapWindow::setupUI()
{
    UI.setupUi(this);
    SimoxGui::ViewerFactoryPtr factory = SimoxGui::ViewerFactory::fromName(VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType(), nullptr);
    THROW_VR_EXCEPTION_IF(!factory,"Could not create ViewerFactory.");
    viewer = factory->createViewer(UI.frameViewer);

    viewer->viewAll();

    connect(UI.pushButtonObjectRandom, SIGNAL(clicked()), this, SLOT(setObjectRandom()));

    connect(UI.checkBoxRobot, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxObject, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxReachabilityMapVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));

    connect(UI.radioButtonAllGrasps, SIGNAL(clicked()), this, SLOT(selectGrasp()));
    connect(UI.radioButtonOneGrasp, SIGNAL(clicked()), this, SLOT(selectGrasp()));
    connect(UI.comboBoxGrasp, SIGNAL(currentIndexChanged(int)), this, SLOT(selectGrasp()));
    connect(UI.comboBoxEEF, SIGNAL(currentIndexChanged(int)), this, SLOT(selectEEF()));

}

QString ReachabilityMapWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void ReachabilityMapWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::map<ModelJointPtr,float> configMap;
    for (ModelJointPtr joint : robot->getJoints())
    {
        configMap[joint] = 0.0f;
    }
    robot->setJointValues(configMap);
}

void ReachabilityMapWindow::updateVisu()
{
    if (UI.checkBoxRobot->isChecked())
    {
        buildRobotVisu();
    }
    else
    {
        viewer->clearLayer(robotVisuLayer);
    }

    if (UI.checkBoxGrasps->isChecked())
    {
        buildGraspVisu();
    }
    else
    {
        viewer->clearLayer(graspVisuLayer);
    }

    if (UI.checkBoxObject->isChecked())
    {
        buildObjectVisu();
    }
    else
    {
        viewer->clearLayer(objectVisuLayer);
    }

    if (UI.checkBoxReachabilityVisu->isChecked())
    {
        buildReachVisu();
    }
    else
    {
        viewer->clearLayer(reachVisuLayer);
    }

    if (UI.checkBoxReachabilityMapVisu->isChecked())
    {
        buildReachGridVisu();
    }
    else
    {
        viewer->clearLayer(reachMapVisuLayer);
    }
}


void ReachabilityMapWindow::buildReachVisu()
{
    if (!robot || !reachSpace || !graspObject || !eef)
    {
        return;
    }

    viewer->clearLayer(reachVisuLayer);

    GraspSetPtr gs = graspObject->getGraspSet(eef);

    if (!gs || gs->getSize() == 0)
    {
        return;
    }

    Eigen::Matrix4f pose = graspObject->getGlobalPose();

    if (UI.radioButtonOneGrasp->isChecked())
    {
        QString qs(UI.comboBoxGrasp->currentText());
        std::string s(qs.toLatin1());
        GraspPtr g = gs->getGrasp(s);

        if (g)
        {
            pose = g->getTcpPoseGlobal(graspObject->getGlobalPose());
        }
    }

    WorkspaceRepresentation::WorkspaceCut2DPtr cutData = reachSpace->createCut(pose,reachSpace->getDiscretizeParameterTranslation(), false);
    int maxCoeff = cutData->entries.maxCoeff();
    VR_INFO << "Max coeff:" << maxCoeff << endl;

    SoNode *node = CoinVisualizationFactory::getCoinVisualization(cutData, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), Eigen::Vector3f::UnitZ(), maxCoeff);
    CoinVisualizationPtr v(new CoinVisualization(VisualizationNodePtr(new CoinVisualizationNode(node))));


    if (v)
    {
        if (reachSpace->getBaseNode())
        {
            Eigen::Matrix4f gp = reachSpace->getBaseNode()->getGlobalPose();
            VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(v, gp);
        }

        viewer->addVisualization(reachVisuLayer, "reachability", v);
    }

}

void ReachabilityMapWindow::buildRobotVisu()
{
    viewer->clearLayer(robotVisuLayer);

    if (!robot)
    {
        return;
    }

    VisualizationPtr visualization = robot->getVisualization();

    if (visualization)
    {
        viewer->addVisualization(robotVisuLayer, "robot", visualization);
    }
}

void ReachabilityMapWindow::buildObjectVisu()
{
    viewer->clearLayer(objectVisuLayer);

    if (!graspObject)
    {
        return;
    }

    VisualizationPtr visuObject = graspObject->getVisualization();

    if (visuObject)
    {
        viewer->addVisualization(objectVisuLayer, "object", visuObject);
    }

    if (environment)
    {
        VisualizationPtr visuEnv = environment->getVisualization();

        if (visuEnv)
        {
            viewer->addVisualization(objectVisuLayer, "environment", visuEnv);
        }
    }
}

void ReachabilityMapWindow::buildGraspVisu()
{
    if (!robot || !graspObject || !eef)
    {
        return;
    }

    GraspSetPtr gs = graspObject->getGraspSet(eef);

    if (!gs || gs->getSize() == 0)
    {
        return;
    }

    viewer->clearLayer(graspVisuLayer);
    if (UI.radioButtonOneGrasp->isChecked())
    {
        QString qs(UI.comboBoxGrasp->currentText());
        std::string s(qs.toLatin1());
        GraspPtr g = gs->getGrasp(s);

        if (!g)
        {
            return;
        }

        // TODO here, we have to use coin explicitly because the visufactory-interface does not have a grasp render method.
        SoNode* node = CoinVisualizationFactory::CreateGraspVisualization(g, eef, graspObject->getGlobalPose());
        CoinVisualizationPtr v(new CoinVisualization(VisualizationNodePtr(new CoinVisualizationNode(node))));

        if (v)
        {
            viewer->addVisualization(graspVisuLayer, "grasp", v);
        }
    }
    else
    {
        VisualizationPtr v = VisualizationFactory::getGlobalVisualizationFactory()->createGraspSetVisualization(gs, eef, graspObject->getGlobalPose(), ModelLink::Full);
        if (v)
        {
            viewer->addVisualization(graspVisuLayer, "grasp", v);
        }
    }
}

void ReachabilityMapWindow::buildReachGridVisu()
{
    if (!robot || !reachGrid)
    {
        return;
    }

    viewer->clearLayer(reachMapVisuLayer);
    // TODO here, we have to use coin explicitly because the visufactory-interface does not have a reach grid render method.
    SoNode* node = CoinVisualizationFactory::getCoinVisualization(reachGrid, VirtualRobot::ColorMap::eHot, true);
    CoinVisualizationPtr v(new CoinVisualization(VisualizationNodePtr(new CoinVisualizationNode(node))));

    if (v)
    {
        viewer->addVisualization(reachMapVisuLayer, "reachmap", v);
    }
}

void ReachabilityMapWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int ReachabilityMapWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void ReachabilityMapWindow::quit()
{
    std::cout << "ReachabilityMapWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void ReachabilityMapWindow::updateEEFBox()
{
    UI.comboBoxEEF->clear();

    if (!robot)
    {
        selectGrasp();
        return;
    }

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();

    for (unsigned int i = 0; i < eefs.size(); i++)
    {
        UI.comboBoxEEF->addItem(QString(eefs[i]->getName().c_str()));
    }

    selectEEF(0);
}

void ReachabilityMapWindow::selectGrasp()
{
    if (!grasps)
    {
        return;
    }

    viewer->clearLayer(graspVisuLayer);
    viewer->clearLayer(reachMapVisuLayer);
    viewer->clearLayer(reachVisuLayer);

    if (UI.radioButtonAllGrasps->isChecked())
    {
        buildReachMapAll();
    }
    else
    {
        GraspPtr g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
        buildReachMap(g);
    }

    updateVisu();
}


void ReachabilityMapWindow::selectEEF()
{
    selectEEF(UI.comboBoxEEF->currentIndex());
}

void ReachabilityMapWindow::selectEEF(int nr)
{
    eef.reset();
    grasps.reset();
    UI.comboBoxGrasp->clear();
    viewer->clearLayer(graspVisuLayer);

    if (!robot)
    {
        selectGrasp();
        return;
    }

    cout << "Selecting EEF nr " << nr << endl;

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
    std::string tcp = "<not set>";

    if (nr < 0 || nr >= (int)eefs.size())
    {
        return;
    }

    eef = eefs[nr];

    if (graspObject)
    {
        grasps = graspObject->getGraspSet(eef);

        if (grasps)
        {
            for (unsigned int i = 0; i < grasps->getSize(); i++)
            {
                UI.comboBoxGrasp->addItem(QString(grasps->getGrasp(i)->getName().c_str()));
            }
        }
    }

    selectGrasp();
}

void ReachabilityMapWindow::selectEEF(std::string& eef)
{
    if (!robot)
    {
        return;
    }

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();

    for (size_t i = 0; i < eefs.size(); i++)
    {
        if (eefs[i]->getName() == eef)
        {
            selectEEF((int)i);
            UI.comboBoxEEF->setCurrentIndex((int)i);
        }
    }
}

void ReachabilityMapWindow::loadRobot()
{
    viewer->clearLayer(robotVisuLayer);
    cout << "Loading robot from " << robotFile << endl;

    try
    {
        robot = ModelIO::loadModel(robotFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    //Eigen::Matrix4f gp = MathTools::rpy2eigen4f(0,0,M_PI/2.0);
    Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
    //gp(0,3) = 3000;
    robot->setGlobalPose(gp);

    updateEEFBox();

    // build visualization
    updateVisu();
    viewer->viewAll();
}

void ReachabilityMapWindow::loadReachFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    reachFile = filename;
    bool success = false;

    // The following is one of the reasons why we should refactor the workspace api.
    // 1st try to load as reachability file
    try
    {
        reachSpace.reset(new Reachability(robot));
        reachSpace->load(reachFile);
        success = true;

        VR_INFO << "Map '" << reachFile << "' loaded as Reachability map" << std::endl;
    }
    catch (...)
    {
        VR_ERROR << "Coulkd not load reachability file..." << endl;
    }

    // 2nd try to load as manipulability file
    if (!success)
    {
        try
        {
            reachSpace.reset(new Manipulability(robot));
            reachSpace->load(reachFile);
            success = true;

            VR_INFO << "Map '" << reachFile << "' loaded as Manipulability map" << std::endl;
        }
        catch (...)
        {
        }
    }


    reachSpace->print();
    /*if (reachSpace->getNodeSet())
    {
        cout << "Using RNS: " << reachSpace->getNodeSet()->getName() << endl;
        for (size_t i=0;i<robotNodeSets.size();i++)
        {
            cout << "checking " << robotNodeSets[i]->getName() << endl;
            if (robotNodeSets[i] == reachSpace->getNodeSet())
            {
                cout << "Found RNS.." << endl;
                //selectRNS(i);
            }
        }
    }*/
}
void ReachabilityMapWindow::setObjectRandom()
{
    if (graspObject)
    {
        Eigen::Matrix4f gp;
        gp.setIdentity();
        gp(0, 3) = 50.0f + (float)(rand() % 1100);
        gp(1, 3) = -50.0f - (float)(rand() % 720);
        gp(2, 3) = 1030.0f;

        graspObject->setGlobalPose(gp);
        selectGrasp();
    }
}
void ReachabilityMapWindow::setupEnvironment()
{
    std::string objectFile("objects/Table.xml");

    if (!RuntimeEnvironment::getDataFileAbsolute(objectFile))
    {
        VR_ERROR << "No path to " << objectFile << endl;
        return;
    }

    try
    {
        environment = ObjectIO::loadManipulationObject(objectFile);
    }
    catch (VirtualRobotException e)
    {
        VR_ERROR << "Could not load " << objectFile << endl;
        return;
    }

    if (!environment)
    {
        return;
    }

    Eigen::Matrix4f gp;
    gp.setIdentity();
    environment->setGlobalPose(gp);
    setObjectRandom();

}

void ReachabilityMapWindow::loadObjectFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    objectFile = filename;

    try
    {
        graspObject = ObjectIO::loadManipulationObject(filename);
    }
    catch (VirtualRobotException e)
    {
        VR_ERROR << "Could not load " << filename << endl;
        return;
    }
}

bool ReachabilityMapWindow::buildReachMapAll()
{
    viewer->clearLayer(reachMapVisuLayer);

    if (!grasps)
    {
        return false;
    }

    Eigen::Vector3f minBB, maxBB;
    reachSpace->getWorkspaceExtends(minBB, maxBB);
    reachGrid.reset(new WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reachSpace->getDiscretizeParameterTranslation()));

    Eigen::Matrix4f gp = graspObject->getGlobalPose();
    reachGrid->setGridPosition(gp(0, 3), gp(1, 3));

    for (int i = 0; i < (int)grasps->getSize(); i++)
    {
        GraspPtr g = grasps->getGrasp(i);
        reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode());
    }

    updateVisu();
    return true;
}

bool ReachabilityMapWindow::buildReachMap(VirtualRobot::GraspPtr g)
{
    viewer->clearLayer(reachMapVisuLayer);
    Eigen::Vector3f minBB, maxBB;
    reachSpace->getWorkspaceExtends(minBB, maxBB);
    reachGrid.reset(new WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reachSpace->getDiscretizeParameterTranslation()));

    Eigen::Matrix4f gp = graspObject->getGlobalPose();
    reachGrid->setGridPosition(gp(0, 3), gp(1, 3));

    reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode());

    updateVisu();
    return true;

}
