
#include "GraspRrtWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Model/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/GraspRrt.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/RrtWorkspaceVisualization.h>

#include <QFileDialog>

#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>


#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    #include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>

// need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif


using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 200.0f;

GraspRrtWindow::GraspRrtWindow(const std::string& sceneFile, const std::string& sConf, const std::string& goalObject,
                               const std::string& rns, const std::string& rnsB, const std::string& eefName, const std::string& eefNameB,
                               const std::string& colModelRob1, const std::string& colModelRob1B, const std::string& colModelRob2, const std::string& colModelRob2B,
                               const std::string& colModelEnv)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;

    planSetA.rns = rns;
    planSetA.eef = eefName;
    planSetA.colModelRob1 = colModelRob1;
    planSetA.colModelRob2 = colModelRob2;
    planSetA.colModelEnv = colModelEnv;

    planSetB.rns = rnsB;
    planSetB.eef = eefNameB;
    planSetB.colModelRob1 = colModelRob1B;
    planSetB.colModelRob2 = colModelRob2B;
    planSetB.colModelEnv = colModelEnv;
    setupUI();

    loadScene();

    selectPlanSet(0);

    selectStart(sConf);
    selectTargetObject(goalObject);

    if (sConf != "")
    {
        UI.comboBoxStart->setEnabled(false);
    }

    if (goalObject != "")
    {
        UI.comboBoxGoal->setEnabled(false);
    }

    if (rns != "")
    {
        UI.comboBoxRNS->setEnabled(false);
    }

    if (colModelRob1 != "")
    {
        UI.comboBoxColModelRobot->setEnabled(false);
    }

    if (colModelRob2 != "")
    {
        UI.comboBoxColModelRobotStatic->setEnabled(false);
    }

    viewer->viewAll();
}


GraspRrtWindow::~GraspRrtWindow()
{
}



void GraspRrtWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    UI.radioButtonSolution->setChecked(true);

    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadSceneWindow()));
    connect(UI.checkBoxShowSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowSolutionOpti, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowRRT, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.horizontalSliderPos, SIGNAL(sliderMoved(int)), this, SLOT(sliderSolution(int)));
    connect(UI.radioButtonSolution, SIGNAL(clicked()), this, SLOT(solutionSelected()));
    connect(UI.radioButtonSolutionOpti, SIGNAL(clicked()), this, SLOT(solutionSelected()));

    connect(UI.comboBoxStart, SIGNAL(activated(int)), this, SLOT(selectStart(int)));
    connect(UI.comboBoxGoal, SIGNAL(activated(int)), this, SLOT(selectTargetObject(int)));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxColModelRobot, SIGNAL(activated(int)), this, SLOT(selectColModelRobA(int)));
    connect(UI.comboBoxColModelRobotStatic, SIGNAL(activated(int)), this, SLOT(selectColModelRobB(int)));
    connect(UI.comboBoxColModelEnv, SIGNAL(activated(int)), this, SLOT(selectColModelEnv(int)));

    connect(UI.pushButtonGraspPose, SIGNAL(clicked()), this, SLOT(testGraspPose()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
}


void GraspRrtWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GraspRrtWindow::buildVisu()
{
    viewer->clearLayer("scene");
    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::Collision : ModelLink::Full;

    VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
    if (!f)
        return;

    if (scene)
    {
        VisualizationSetPtr v = VisualizationFactory::getGlobalVisualizationFactory()->getVisualization(scene, colModel);
        viewer->addVisualization("scene", "scene", v);
    }

    viewer->clearLayer("grasps");

    if (UI.checkBoxGrasps->isChecked() && eef && grasps.size()>0)
    {
        GraspSetPtr gs(new GraspSet("tmp", robot->getName(), eef->getName(), grasps));
        VisualizationSetPtr v = VisualizationFactory::getGlobalVisualizationFactory()->createGraspSetVisualization(gs, eef, targetObject->getGlobalPose(), ModelLink::Full);
        viewer->addVisualization("scene", "scene", v);
    }

/*
        SoSeparator* eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);

        for (size_t i = 0; i < grasps.size(); i++)
        {

            Eigen::Matrix4f m = grasps[i]->getTcpPoseGlobal(targetObject->getGlobalPose());
            SoSeparator* sep1 = new SoSeparator();
            SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(m);
            sep1->addChild(mt);
            sep1->addChild(eefVisu);
            graspsSep->addChild(sep1);
        }
    }
*/
    buildRRTVisu();

    redraw();
}

int GraspRrtWindow::main()
{
    viewer->start(this);
    return 0;
}


void GraspRrtWindow::quit()
{
    std::cout << "GraspRrtWindow: Closing" << std::endl;
    this->close();
    viewer->stop();
}

void GraspRrtWindow::loadSceneWindow()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));

    if (fi == "")
    {
        return;
    }

    sceneFile = std::string(fi.toLatin1());
    loadScene();
}

void GraspRrtWindow::loadScene()
{
    robot.reset();
    scene = SceneIO::loadScene(sceneFile);

    if (!scene)
    {
        VR_ERROR << " no scene ..." << endl;
        return;
    }

    std::vector< RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "Need exactly 1 robot" << endl;
        return;
    }

    robot = robots[0];
    //robotStart = robot->clone("StartConfig");
    //robotGoal = robot->clone("GoalConfig");

    // setup start Config combo box
    configs = scene->getRobotConfigs(robot);

    if (configs.size() < 1)
    {
        VR_ERROR << "Need at least 1 Robot Configurations" << endl;
        return;
    }

    UI.comboBoxStart->clear();

    for (size_t i = 0; i < configs.size(); i++)
    {
        QString qtext = configs[i]->getName().c_str();
        UI.comboBoxStart->addItem(qtext);

    }

    UI.comboBoxStart->setCurrentIndex(0);
    selectStart(0);

    // setup target object combo box
    obstacles = scene->getObstacles();

    if (obstacles.size() < 1)
    {
        VR_ERROR << "Need at least 1 Obstacle (target object)" << endl;
        return;
    }

    UI.comboBoxGoal->clear();

    for (size_t i = 0; i < obstacles.size(); i++)
    {
        QString qtext = obstacles[i]->getName().c_str();
        UI.comboBoxGoal->addItem(qtext);

    }

    UI.comboBoxGoal->setCurrentIndex(0);
    //selectTargetObject(0);

    // steup scene objects (col models env)
    std::vector<ModelSetPtr> soss = scene->getModelSets();
    UI.comboBoxColModelEnv->clear();
    QString qtext;

    for (size_t i = 0; i < soss.size(); i++)
    {
        qtext = soss[i]->getName().c_str();
        UI.comboBoxColModelEnv->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelEnv->addItem(qtext);

    //setup eefs
    if (!robot->hasEndEffector(planSetA.eef))
    {
        VR_ERROR << "EEF with name " << planSetA.eef << " not known?!" << endl;
        return;
    }

    UI.comboBoxEEF->clear();
    qtext = planSetA.eef.c_str();
    UI.comboBoxEEF->addItem(qtext);

    if (robot->hasEndEffector(planSetB.eef))
    {
        qtext = planSetB.eef.c_str();
        UI.comboBoxEEF->addItem(qtext);
    }

    /*std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
    for (size_t i=0; i < eefs.size(); i++)
    {
        qtext = planSetA.eef.c_str();
        UI.comboBoxEEF->addItem(qtext);
    }*/

    // Setup robot node sets and col models
    std::vector<JointSetPtr> jointsets = robot->getJointSets();
    std::vector<LinkSetPtr> linksets = robot->getLinkSets();
    UI.comboBoxColModelRobot->clear();
    UI.comboBoxColModelRobotStatic->clear();
    UI.comboBoxRNS->clear();

    for (size_t i = 0; i < jointsets.size(); i++)
    {
        qtext = jointsets[i]->getName().c_str();
        UI.comboBoxRNS->addItem(qtext);
    }
    for (size_t i = 0; i < linksets.size(); i++)
    {
        qtext = linksets[i]->getName().c_str();
        UI.comboBoxColModelRobot->addItem(qtext);
        UI.comboBoxColModelRobotStatic->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelRobot->addItem(qtext);
    UI.comboBoxColModelRobotStatic->addItem(qtext);
    robot->setThreadsafe(false);
    buildVisu();
}

void GraspRrtWindow::selectStart(const std::string& conf)
{
    for (size_t i = 0; i < configs.size(); i++)
    {
        if (configs[i]->getName() == conf)
        {
            selectStart(i);
            UI.comboBoxStart->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No configuration with name <" << conf << "> found..." << endl;
}

void GraspRrtWindow::selectTargetObject(const std::string& conf)
{
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i]->getName() == conf)
        {
            selectTargetObject(i);
            UI.comboBoxGoal->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No obstacle with name <" << conf << "> found..." << endl;
}

void GraspRrtWindow::selectRNS(const std::string& rns)
{
    if (!robot)
    {
        return;
    }

    std::vector< JointSetPtr > rnss = robot->getJointSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == rns)
        {
            selectRNS(i);
            UI.comboBoxRNS->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No joint set with name <" << rns << "> found..." << endl;
}

void GraspRrtWindow::selectEEF(const std::string& eefName)
{
    if (!robot)
    {
        return;
    }

    if (eefName == planSetA.eef && UI.comboBoxEEF->count() > 0)
    {
        //selectEEF(0);
        UI.comboBoxEEF->setCurrentIndex(0);
        this->eef = robot->getEndEffector(eefName);
    }
    else if (eefName == planSetB.eef && UI.comboBoxEEF->count() > 1)
    {
        //selectEEF(1);
        UI.comboBoxEEF->setCurrentIndex(1);
        this->eef = robot->getEndEffector(eefName);
    }
    else
    {
        VR_ERROR << "No eef with name <" << eefName << "> found..." << endl;
        return;
    }
}

void GraspRrtWindow::selectColModelRobA(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< LinkSetPtr > rnss = robot->getLinkSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobA(i);
            UI.comboBoxColModelRobot->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}

void GraspRrtWindow::selectColModelRobB(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< LinkSetPtr > rnss = robot->getLinkSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobB(i);
            UI.comboBoxColModelRobotStatic->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}

void GraspRrtWindow::selectColModelEnv(const std::string& colModel)
{
    if (!scene)
    {
        return;
    }

    std::vector<ModelSetPtr> rnss = scene->getModelSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelEnv(i);
            UI.comboBoxColModelEnv->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No scene object set with name <" << colModel << "> found..." << endl;
}

void GraspRrtWindow::selectStart(int nr)
{
    if (nr < 0 || nr >= (int)configs.size())
    {
        return;
    }

    //if (robotStart)
    //  configs[nr]->applyToRobot(robotStart);
    if (robot)
    {
        robot->setJointValues(configs[nr]);
    }

    //configs[nr]->setJointValues();
    if (rns)
    {
        rns->getJointValues(startConfig);
    }
}

void GraspRrtWindow::selectTargetObject(int nr)
{
    if (nr < 0 || nr >= (int)obstacles.size())
    {
        return;
    }

    //if (robotGoal)
    //  configs[nr]->applyToRobot(robotGoal);
    //configs[nr]->setJointValues();
    //if (rns)
    //  rns->getJointValues(goalConfig);
    targetObject = obstacles[nr];
    graspQuality.reset(new GraspPlanning::GraspQualityMeasureWrenchSpace(targetObject));
    int points = 400;
#ifndef NDEBUG
    points = 100;
#endif
    graspQuality->calculateOWS(points);
}

void GraspRrtWindow::selectRNS(int nr)
{
    this->rns.reset();

    if (!robot)
    {
        return;
    }

    std::vector< JointSetPtr > rnss = robot->getJointSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->rns = rnss[nr];
}

void GraspRrtWindow::selectEEF(int nr)
{
    this->eef.reset();

    if (!robot)
    {
        return;
    }

    selectPlanSet(nr);
    /*  std::vector< EndEffectorPtr > eefs = robot->getEndEffectors();
        if (nr<0 || nr>=(int)eefs.size())
            return;
        this->eef = eefs[nr];*/
}

void GraspRrtWindow::selectColModelRobA(int nr)
{
    colModelRobA.reset();

    if (!robot)
    {
        return;
    }

    std::vector< LinkSetPtr > rnss = robot->getLinkSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobA = robot->getLinkSet(rnss[nr]->getName());
}

void GraspRrtWindow::selectColModelRobB(int nr)
{
    colModelRobB.reset();

    if (!robot)
    {
        return;
    }

    std::vector< LinkSetPtr > rnss = robot->getLinkSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobB = robot->getLinkSet(rnss[nr]->getName());
}

void GraspRrtWindow::selectColModelEnv(int nr)
{
    colModelEnv.reset();

    if (!scene)
    {
        return;
    }

    std::vector< ModelSetPtr > rnss = scene->getModelSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelEnv = scene->getModelSet(rnss[nr]->getName());
}

void GraspRrtWindow::buildRRTVisu()
{
    viewer->clearLayer("rrt");

    if (!cspace || !robot)
    {
        return;
    }


    MotionPlanning::RrtWorkspaceVisualizationPtr w;
    #ifdef Simox_USE_COIN_VISUALIZATION
        w.reset(new MotionPlanning::CoinRrtWorkspaceVisualization(robot, cspace, eef->getGCP()->getName()));
    #else
        VR_ERROR << "NO VISUALIZATION IMPLEMENTATION SPECIFIED..." << endl;
    #endif


    if (UI.checkBoxShowRRT->isChecked())
    {
        if (tree)
        {
            w->setCustomColor(0.5f, 0.5f, 0.5f);
            w->colorizeTreeNodes(2, MotionPlanning::RrtWorkspaceVisualization::eRed);
            w->addTree(tree, MotionPlanning::RrtWorkspaceVisualization::eCustom);
        }
    }

    if (UI.checkBoxShowSolution->isChecked() && solution)
    {
        w->addCSpacePath(solution);
    }

    if (UI.checkBoxShowSolutionOpti->isChecked() && solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, MotionPlanning::RrtWorkspaceVisualization::eGreen);
    }

    w->addConfiguration(startConfig, MotionPlanning::RrtWorkspaceVisualization::eGreen, 3.0f);
    VisualizationSetPtr wv = w->getVisualization();
    if (wv)
    {
        viewer->addVisualization("rrt","solution", wv);
    }
}

void GraspRrtWindow::testInit()
{
    // setup collision detection
    if (!test_cspace)
    {

        CDManagerPtr cdm(new CDManager());

        if (colModelRobA)
        {
            cdm->addCollisionModel(colModelRobA);
        }

        if (colModelRobB)
        {
            cdm->addCollisionModel(colModelRobB);
        }

        if (colModelEnv)
        {
            cdm->addCollisionModel(colModelEnv);
        }

        test_cspace.reset(new MotionPlanning::CSpaceSampled(robot, cdm, rns, 500000));
    }

    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    test_cspace->setSamplingSize(sampl);
    test_cspace->setSamplingSizeDCD(samplDCD);
    float minGraspScore = (float)UI.doubleSpinBoxMinGraspScore->value();

    std::vector<ModelPtr> models = colModelEnv->getModels();
    test_graspRrt.reset(new MotionPlanning::GraspRrt(test_cspace, eef, targetObject, graspQuality, models, 0.1f, minGraspScore));

    test_graspRrt->setStart(startConfig);
    test_graspRrt->init();
}

void GraspRrtWindow::testGraspPose()
{
    if (!robot || !rns || !eef || !graspQuality)
    {
        return;
    }

    if (!test_cspace || !test_graspRrt)
    {
        testInit();
    }

    // create taregt on objects surface
    Eigen::Matrix4f globalGrasp;
    Eigen::VectorXf c(rns->getSize());
    //cspace->getRandomConfig(c);
    //rns->setJointValues(c);
    rns->getJointValues(c);
    test_graspRrt->calculateGlobalGraspPose(c, globalGrasp);

    /*VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(10, 10, 10);
    o->setGlobalPose(globalGrasp);
    //o->showCoordinateSystem(true);
    graspsSep->removeAllChildren();
    graspsSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(o, VirtualRobot::ModelLink::Full));*/

    // move towards object
    Eigen::Matrix4f p = eef->getGCP()->getGlobalPose();
    test_graspRrt->createWorkSpaceSamplingStep(p, globalGrasp, c);
    rns->setJointValues(c);
}

void GraspRrtWindow::plan()
{
    if (!robot || !rns || !eef || !graspQuality)
    {
        return;
    }

    // setup collision detection
    CDManagerPtr cdm(new CDManager());

    if (colModelRobA)
    {
        cdm->addCollisionModel(colModelRobA);
    }

    if (colModelRobB)
    {
        cdm->addCollisionModel(colModelRobB);
    }

    if (colModelEnv)
    {
        cdm->addCollisionModel(colModelEnv);
    }

    cdm->addCollisionModel(targetObject);
    cspace.reset(new MotionPlanning::CSpaceSampled(robot, cdm, rns, 500000));
    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    cspace->setSamplingSize(sampl);
    cspace->setSamplingSizeDCD(samplDCD);
    float minGraspScore = (float)UI.doubleSpinBoxMinGraspScore->value();

    std::vector<ModelPtr> models = colModelEnv->getModels();
    MotionPlanning::GraspRrtPtr graspRrt(new MotionPlanning::GraspRrt(cspace, eef, targetObject, graspQuality, models , 0.1f, minGraspScore));

    graspRrt->setStart(startConfig);

    //bool planOk = false;
    bool planOK = graspRrt->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = graspRrt->getSolution();
        MotionPlanning::ShortcutProcessorPtr postProcessing(new MotionPlanning::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = graspRrt->getTree();
        std::vector<MotionPlanning::GraspRrt::GraspInfo, Eigen::aligned_allocator<MotionPlanning::GraspRrt::GraspInfo> > vStoreGraspInfo;
        graspRrt->getGraspInfoResult(vStoreGraspInfo);
        grasps.clear();

        for (size_t i = 0; i < vStoreGraspInfo.size(); i++)
        {
            cout << "processing grasp " << i << endl;
            VirtualRobot::GraspPtr g(new VirtualRobot::Grasp("GraspRrt Grasp", robot->getType(), eef->getName(), vStoreGraspInfo[i].handToObjectTransform, "GraspRrt", vStoreGraspInfo[i].graspScore));
            grasps.push_back(g);
        }
    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void GraspRrtWindow::colModel()
{
    buildVisu();
}

void GraspRrtWindow::solutionSelected()
{
    sliderSolution(UI.horizontalSliderPos->sliderPosition());
}

void GraspRrtWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    MotionPlanning::CSpacePathPtr s = solution;

    if (UI.radioButtonSolutionOpti->isChecked() && solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    rns->setJointValues(iPos);
    redraw();
}

void GraspRrtWindow::redraw()
{
    UI.frameViewer->update();
    this->update();
}


void GraspRrtWindow::selectPlanSet(int nr)
{
    if (nr == 0)
    {
        selectRNS(planSetA.rns);
        selectEEF(planSetA.eef);
        selectColModelRobA(planSetA.colModelRob1);
        selectColModelRobB(planSetA.colModelRob2);
        selectColModelEnv(planSetA.colModelEnv);
        selectStart(UI.comboBoxStart->currentIndex());
    }
    else
    {
        selectRNS(planSetB.rns);
        selectEEF(planSetB.eef);
        selectColModelRobA(planSetB.colModelRob1);
        selectColModelRobB(planSetB.colModelRob2);
        selectColModelEnv(planSetB.colModelEnv);
        selectStart(UI.comboBoxStart->currentIndex());
    }
}


void GraspRrtWindow::closeEEF()
{
    if (eef)
    {
        eef->closeActors(targetObject);
    }

    redraw();
}

void GraspRrtWindow::openEEF()
{
    if (eef)
    {
        eef->openActors();
    }

    redraw();
}

