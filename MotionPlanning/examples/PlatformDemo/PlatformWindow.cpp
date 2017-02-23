
#include "PlatformWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/Obstacle.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/BiRrt.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 200.0f;

PlatformWindow::PlatformWindow(const std::string& sceneFile,
                               const std::string& rns,
                               const std::string& colModelRob,
                               const std::string& colModelEnv)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;

    allSep = new SoSeparator;
    allSep->ref();
    sceneFileSep = new SoSeparator;
    rrtSep = new SoSeparator;

    allSep->addChild(sceneFileSep);
    allSep->addChild(rrtSep);

    planSetA.rns = rns;
    planSetA.colModelRob = colModelRob;
    planSetA.colModelEnv = colModelEnv;

    setupUI();

    loadScene();

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}


PlatformWindow::~PlatformWindow()
{
    allSep->unref();
}


void PlatformWindow::timerCB(void* data, SoSensor* /*sensor*/)
{
    PlatformWindow* ikWindow = static_cast<PlatformWindow*>(data);
    ikWindow->redraw();
}


void PlatformWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);

    viewer->setAntialiasing(true, 4);

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(allSep);
    viewer->viewAll();

    UI.radioButtonSolution->setChecked(true);

    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadSceneWindow()));
    connect(UI.checkBoxShowSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowSolutionOpti, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowRRT, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.horizontalSliderPos, SIGNAL(sliderMoved(int)), this, SLOT(sliderSolution(int)));
    connect(UI.radioButtonSolution, SIGNAL(clicked()), this, SLOT(solutionSelected()));
    connect(UI.radioButtonSolutionOpti, SIGNAL(clicked()), this, SLOT(solutionSelected()));
}




void PlatformWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void PlatformWindow::buildVisu()
{
    sceneFileSep->removeAllChildren();

    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (scene)
    {
        visualization = scene->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = NULL;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            sceneFileSep->addChild(visualisationNode);
        }
    }

    buildRRTVisu();

    redraw();
}

int PlatformWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void PlatformWindow::quit()
{
    std::cout << "PlatformWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void PlatformWindow::loadSceneWindow()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));

    if (fi == "")
    {
        return;
    }

    sceneFile = std::string(fi.toLatin1());
    loadScene();
}

void PlatformWindow::loadScene()
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

    // setup target object combo box
    obstacles = scene->getObstacles();

    if (obstacles.size() < 1)
    {
        VR_ERROR << "Need at least 1 Obstacle (target object)" << endl;
        return;
    }

    // steup scene objects (col models env)
    std::vector<SceneObjectSetPtr> soss = scene->getSceneObjectSets();

    // Setup robot node sets and col models
    std::vector<RobotNodeSetPtr> rnss = robot->getRobotNodeSets();

    rns = robot->getRobotNodeSet(planSetA.rns);
    colModelRob = robot->getRobotNodeSet(planSetA.colModelRob);
    colModelEnv = scene->getSceneObjectSet(planSetA.colModelEnv);

    bool startOK = false;
    bool goalOK = false;

    // setup start Config combo box
    configs = scene->getRobotConfigs(robot);

    for (auto c:configs)
    {
        if (c->getName() == "start")
        {
            c->setJointValues(robot);
            Eigen::VectorXf conf(3);
            rns->getJointValues(conf);
            setStart(conf);
            startOK = true;
        }
        if (c->getName() == "goal")
        {
            c->setJointValues(robot);
            Eigen::VectorXf conf(3);
            rns->getJointValues(conf);
            setGoal(conf);
            goalOK = true;
        }
    }
    if (!startOK)
    {
        THROW_VR_EXCEPTION("Missing configuartion 'start' in scene definition");
    }
    if (!goalOK)
    {
        THROW_VR_EXCEPTION("Missing configuartion 'goal' in scene definition");
    }

    robot->setThreadsafe(false);
    buildVisu();
}


void PlatformWindow::setStart(Eigen::VectorXf &startConf)
{
    if (rns)
        rns->setJointValues(startConf);

    startConfig = startConf;
}

void PlatformWindow::setGoal(Eigen::VectorXf &goalConf)
{

    goalConfig = goalConf;
}


void PlatformWindow::selectColModelRob(const std::string &name)
{
    colModelRob.reset();

    if (!robot)
    {
        return;
    }

    this->colModelRob = robot->getRobotNodeSet(name);
}



void PlatformWindow::selectColModelEnv(const std::string &name)
{
    colModelEnv.reset();

    if (!scene)
    {
        return;
    }

    this->colModelEnv = scene->getSceneObjectSet(name);
}

void PlatformWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!cspace || !robot || !rns)
    {
        return;
    }

    boost::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, rns->getTCP()->getName()));

    if (UI.checkBoxShowRRT->isChecked())
    {
        if (tree)
        {
            w->setCustomColor(0.5f, 0.5f, 0.5f);
            w->colorizeTreeNodes(2, Saba::RrtWorkspaceVisualization::eRed);
            w->addTree(tree, Saba::RrtWorkspaceVisualization::eCustom);
        }
        if (tree2)
        {
            w->setCustomColor(0.5f, 0.5f, 0.5f);
            w->colorizeTreeNodes(2, Saba::RrtWorkspaceVisualization::eRed);
            w->addTree(tree2, Saba::RrtWorkspaceVisualization::eCustom);
        }
    }

    if (UI.checkBoxShowSolution->isChecked() && solution)
    {
        w->addCSpacePath(solution);
    }

    if (UI.checkBoxShowSolutionOpti->isChecked() && solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    w->addConfiguration(startConfig, Saba::CoinRrtWorkspaceVisualization::eGreen, 3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

void PlatformWindow::plan()
{
    if (!robot || !rns)
    {
        return;
    }

    // setup collision detection
    CDManagerPtr cdm(new CDManager());

    if (colModelRob)
    {
        cdm->addCollisionModel(colModelRob);
    }

    if (colModelEnv)
    {
        cdm->addCollisionModel(colModelEnv);
    }

    cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns, 500000));
    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    cspace->setSamplingSize(sampl);
    cspace->setSamplingSizeDCD(samplDCD);
    Eigen::VectorXf w(3);
    w << 1,1,0.001f;
    cspace->setMetricWeights(w);

    Saba::BiRrtPtr rrt(new Saba::BiRrt(cspace));

    rrt->setStart(startConfig);
    rrt->setGoal(goalConfig);

    //bool planOk = false;
    bool planOK = rrt->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = rrt->getSolution();
        Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = rrt->getTree();
        tree2 = rrt->getTree2();
    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void PlatformWindow::colModel()
{
    buildVisu();
}
void PlatformWindow::solutionSelected()
{
    sliderSolution(UI.horizontalSliderPos->sliderPosition());
}
void PlatformWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    Saba::CSpacePathPtr s = solution;

    if (UI.radioButtonSolutionOpti->isChecked() && solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);
    redraw();
}

void PlatformWindow::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}


void PlatformWindow::selectPlanSet(int /*nr*/)
{
    /*if (nr == 0)
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
    }*/
}


