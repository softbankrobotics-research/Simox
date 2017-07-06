
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
#include "MotionPlanning/PostProcessing/ElasticBandProcessor.h"
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
#include <Inventor/nodes/SoUnits.h>

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
    distSep = new SoSeparator;
    forcesSep = new SoSeparator;

    allSep->addChild(sceneFileSep);
    allSep->addChild(rrtSep);
    allSep->addChild(distSep);
    allSep->addChild(forcesSep);

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

    QString t1("Elastic Band");
    QString t2("Shortcut");
    UI.comboBoxSmoothing->addItem(t1);
    UI.comboBoxSmoothing->addItem(t2);

    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadSceneWindow()));
    connect(UI.pushButtonOpti, SIGNAL(clicked()), this, SLOT(optimizeSolutionPressed()));
    connect(UI.checkBoxShowSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowSolutionOpti, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowRRT, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowDistance, SIGNAL(clicked()), this, SLOT(buildVisu()));
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

    distSep->removeAllChildren();

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
    solution.reset();
    solutionOptimized.reset();
    cdmPlayback.reset();
    tree.reset();
    tree2.reset();

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
    std::vector<RobotNodeSetPtr> rnss = robot->getModelNodeSets();

    rns = robot->getModelNodeSet(planSetA.rns);
    colModelRob = robot->getModelNodeSet(planSetA.colModelRob);
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

    this->colModelRob = robot->getModelNodeSet(name);
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

void PlatformWindow::updateDistVisu(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
    distSep->removeAllChildren();
    if (UI.checkBoxShowDistance->isChecked())
    {
        Eigen::Matrix4f from;
        Eigen::Matrix4f to;
        from.setIdentity();
        to.setIdentity();
        from.block(0,3,3,1) = a;
        to.block(0,3,3,1) = b;

        SoNode * c = CoinVisualizationFactory::createCoinLine(from, to, 5.0f, 1.0f, 0.2f, 0.2f);
        distSep->addChild(c);
    }
}

void PlatformWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!cspace || !robot || !rns)
    {
        return;
    }

    std::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, rns->getTCP()->getName()));

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

void PlatformWindow::optimizeSolutionPressed()
{
    int nrSteps = UI.spinBox_smoothing->value();
    postProcessingMethod postProcessing = eElasticBands;
    if (UI.comboBoxSmoothing->currentIndex()==1)
    {
        postProcessing = eShortcuts;
    }
    optimizeSolution(postProcessing, nrSteps);

    sliderSolution(1000);

    buildVisu();
}


void PlatformWindow::showOptizerForces(Saba::ElasticBandProcessorPtr postProcessing, Saba::CSpacePathPtr s)
{
	if (!postProcessing)
		return;
		
	forcesSep->removeAllChildren();

    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    forcesSep->addChild(u);

		
    for (unsigned int i=0; i<s->getNrOfPoints(); i++)
	{
		Eigen::Vector3f internalForce;
        Eigen::Vector3f externalForce;
        Eigen::Vector3f resultingForce;
        postProcessing->getForces(i, internalForce, externalForce);
        resultingForce = internalForce + externalForce;
        Eigen::Vector3f p = postProcessing->getWSpacePoint(s->getPoint(i));
        Eigen::Matrix4f m;
        m.setIdentity();
        m.block(0,3,3,1) = p;

        SoSeparator *sa = new SoSeparator;
        SoSeparator *sb = new SoSeparator;
        SoSeparator *sc = new SoSeparator;
        SoMatrixTransform* t = CoinVisualizationFactory::getMatrixTransform(m);
        sa->addChild(t);
        sb->addChild(t);
        sc->addChild(t);
        float l1 = internalForce.norm() * 5.0f;
        float l2 = externalForce.norm() * 100.0f;
        if (l1<30.0f)
            l1 = 30.0f;
        if (l1>60.0f)
            l1 = 60.0f;
        if (l2<30.0f)
            l2 = 30.0f;
        if (l2>60.0f)
            l2 = 60.0f;
        internalForce.normalize();
        externalForce.normalize();
        resultingForce.normalize();
        SoNode* v1 = CoinVisualizationFactory::CreateArrow(internalForce, l1, 2.0f, VisualizationFactory::Color::Blue());
        SoNode* v2 = CoinVisualizationFactory::CreateArrow(externalForce, l2, 2.0f, VisualizationFactory::Color::Green());
        SoNode* v3 = CoinVisualizationFactory::CreateArrow(resultingForce, l1+l2, 2.0f, VisualizationFactory::Color::Red());
        sa->addChild(v1);
        sb->addChild(v2);
        sc->addChild(v3);
        forcesSep->addChild(sa);
        forcesSep->addChild(sb);
        forcesSep->addChild(sc);
    }
}

void PlatformWindow::optimizeSolution(postProcessingMethod postProcessing, int nrSteps)
{
    VR_INFO << " Smoothing solution with " << nrSteps << " steps " << endl;
    forcesSep->removeAllChildren();
    if (nrSteps<=0)
        return;
    switch (postProcessing)
    {
        case eShortcuts:
        {
            Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solutionOptimized, cspace, false));
            solutionOptimized = postProcessing->optimize(nrSteps);
            break;
        }
        case eElasticBands:
        {
            RobotNodePtr n = colModelRob->getNode(0);
            VR_INFO << "using elsatic band processor with node " << n->getName() << endl;
            Saba::ElasticBandProcessorPtr postProcessing(new Saba::ElasticBandProcessor(solutionOptimized, cspace, n, colModelEnv, false));
            // specific to armar3:
            Eigen::VectorXf w(3);
            w << 1,1,0;
            postProcessing->setWeights(w);
            solutionOptimized = postProcessing->optimize(nrSteps);

            showOptizerForces(postProcessing, solutionOptimized);
            break;
        }
        default:
            VR_INFO << "post processing method nyi" << endl;
    }
    VR_INFO << " Smoothing done" << endl;
}

void PlatformWindow::plan()
{
    if (!robot || !rns)
    {
        return;
    }

    // setup collision detection
    CDManagerPtr cdm(new CDManager());
    cdmPlayback.reset(new CDManager());

    if (colModelRob)
    {
        cdm->addCollisionModel(colModelRob);
        cdmPlayback->addCollisionModel(colModelRob);
    }

    if (colModelEnv)
    {
        cdm->addCollisionModel(colModelEnv);
        cdmPlayback->addCollisionModel(colModelEnv);
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
        solutionOptimized = solution->clone();

        int nrSteps = UI.spinBox_smoothing->value();
        postProcessingMethod postProcessing = eElasticBands;
        if (UI.comboBoxSmoothing->currentIndex()==1)
        {
            postProcessing = eShortcuts;
        }
        optimizeSolution(postProcessing, nrSteps);
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

    std::stringstream d2;
    d2 << setprecision(2) << fixed << "Pos: ";
    for (int i=0;i<rns->getSize();i++)
        d2 << rns->getNode(i)->getJointValue() << ", ";
    QString t2(d2.str().c_str());
    UI.labelPos->setText(t2);

    // update distance
    if (cdmPlayback)
    {
        Eigen::Vector3f P1;
        Eigen::Vector3f P2;
        int trID1;
        int trID2;
        float dist = cdmPlayback->getDistance(P1, P2, trID1, trID2);
        std::stringstream d;
        d << setprecision(2) << fixed << dist;
        QString t(d.str().c_str());
        UI.labelDist->setText(t);

        updateDistVisu(P1,P2);
    }
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



