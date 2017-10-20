
#include "PlatformWindow.h"
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
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/BiRrt.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include "MotionPlanning/PostProcessing/ElasticBandProcessor.h"

#include <QFileDialog>

#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

#ifdef Simox_USE_COIN_VISUALIZATION
    #include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

PlatformWindow::PlatformWindow(const std::string& sceneFile,
                               const std::string& rns,
                               const std::string& colModelRob,
                               const std::string& colModelEnv)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;

    planSetA.rns = rns;
    planSetA.colModelRob = colModelRob;
    planSetA.colModelEnv = colModelEnv;

    setupUI();

    loadScene();

    viewer->viewAll();
}


PlatformWindow::~PlatformWindow()
{
}

void PlatformWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

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

    viewer->clearLayer("scene");

    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::Collision : ModelLink::Full;

    VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
    if (!f)
        return;

    if (scene)
    {
        VisualizationPtr v = VisualizationFactory::getGlobalVisualizationFactory()->getVisualization(scene, colModel);
        viewer->addVisualization("scene", "scenefile", v);
    }

    buildRRTVisu();

    redraw();
}

int PlatformWindow::main()
{
    viewer->start(this);
    return 0;
}


void PlatformWindow::quit()
{
    std::cout << "PlatformWindow: Closing" << std::endl;
    this->close();
    viewer->stop();
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
    //std::vector<LinkSetPtr> soss = scene->getLinkSets();

    // Setup robot node sets and col models
    //std::vector<JointSetPtr> rnss = robot->getJointSets();

    rns = robot->getJointSet(planSetA.rns);
    THROW_VR_EXCEPTION_IF(!rns, "No joint set with name " << planSetA.rns);

    colModelRob = robot->getLinkSet(planSetA.colModelRob);
    colModelEnv = scene->getModelSet(planSetA.colModelEnv);

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

    this->colModelRob = robot->getLinkSet(name);
}



void PlatformWindow::selectColModelEnv(const std::string &name)
{
    colModelEnv.reset();

    if (!scene)
    {
        return;
    }

    this->colModelEnv = scene->getModelSet(name);
}

void PlatformWindow::updateDistVisu(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
    viewer->clearLayer("dist");

    if (UI.checkBoxShowDistance->isChecked())
    {
        Eigen::Matrix4f from;
        Eigen::Matrix4f to;
        from.setIdentity();
        to.setIdentity();
        from.block(0,3,3,1) = a;
        to.block(0,3,3,1) = b;

        VisualizationNodePtr v = VisualizationFactory::getGlobalVisualizationFactory()->createLine(from, to, 5.0f, 1.0f, 0.2f, 0.2f);
        viewer->addVisualization("dist", "rob-obstacle", v);
    }
}

void PlatformWindow::buildRRTVisu()
{
    viewer->clearLayer("rrt");

    if (!cspace || !robot)
    {
        return;
    }

    MotionPlanning::RrtWorkspaceVisualizationPtr w;

#ifdef Simox_USE_COIN_VISUALIZATION
    w.reset(new MotionPlanning::CoinRrtWorkspaceVisualization(robot, cspace, rns->getTCP()->getName()));
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
        if (tree2)
        {
            w->setCustomColor(0.5f, 0.5f, 0.5f);
            w->colorizeTreeNodes(2, MotionPlanning::RrtWorkspaceVisualization::eRed);
            w->addTree(tree2, MotionPlanning::RrtWorkspaceVisualization::eCustom);
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

    VisualizationPtr wv = w->getVisualization();
    if (wv)
    {
        viewer->addVisualization("rrt","workspace", wv);
    }
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


void PlatformWindow::showOptizerForces(MotionPlanning::ElasticBandProcessorPtr postProcessing, MotionPlanning::CSpacePathPtr s)
{
	if (!postProcessing)
		return;
		
    viewer->clearLayer("forces");

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
        VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
        VisualizationNodePtr v1 = f->createArrow(internalForce, l1, 2.0f, VisualizationFactory::Color::Blue());
        VisualizationNodePtr v2 = f->createArrow(internalForce, l1, 2.0f, VisualizationFactory::Color::Blue());
        VisualizationNodePtr v3 = f->createArrow(externalForce, l2, 2.0f, VisualizationFactory::Color::Green());
        VisualizationNodePtr v4 = f->createArrow(resultingForce, l1+l2, 2.0f, VisualizationFactory::Color::Red());

        f->applyDisplacement(v1, m);
        f->applyDisplacement(v2, m);
        f->applyDisplacement(v3, m);
        f->applyDisplacement(v4, m);

        viewer->addVisualization("forces", "f1", v1);
        viewer->addVisualization("forces", "f2", v2);
        viewer->addVisualization("forces", "f3", v3);
        viewer->addVisualization("forces", "f4", v4);

        /*

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
        forcesSep->addChild(sc);*/
    }
}

void PlatformWindow::optimizeSolution(postProcessingMethod postProcessing, int nrSteps)
{
    if (!solutionOptimized)
        return;
    VR_INFO << " Smoothing solution with " << nrSteps << " steps " << endl;
    viewer->clearLayer("forces");

    if (nrSteps<=0)
        return;
    switch (postProcessing)
    {
        case eShortcuts:
        {
            MotionPlanning::ShortcutProcessorPtr postProcessing(new MotionPlanning::ShortcutProcessor(solutionOptimized, cspace, false));
            solutionOptimized = postProcessing->optimize(nrSteps);
            break;
        }
        case eElasticBands:
        {
            ModelLinkPtr n = colModelRob->getNode(0);
            VR_INFO << "using elsatic band processor with node " << n->getName() << endl;
            MotionPlanning::ElasticBandProcessorPtr postProcessing(new MotionPlanning::ElasticBandProcessor(solutionOptimized, cspace, n, colModelEnv, false));
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

    cspace.reset(new MotionPlanning::CSpaceSampled(robot, cdm, rns, 500000));
    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    cspace->setSamplingSize(sampl);
    cspace->setSamplingSizeDCD(samplDCD);
    Eigen::VectorXf w(3);
    w << 1,1,0.001f;
    cspace->setMetricWeights(w);

    MotionPlanning::BiRrtPtr rrt(new MotionPlanning::BiRrt(cspace));

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

    MotionPlanning::CSpacePathPtr s = solution;

    if (UI.radioButtonSolutionOpti->isChecked() && solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    rns->setJointValues(iPos);

    std::stringstream d2;
    d2 << "Pos: ";

    for (unsigned int i=0;i<rns->getSize();i++)
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
        d << fixed << dist;
        QString t(d.str().c_str());
        UI.labelDist->setText(t);

        updateDistVisu(P1,P2);
    }
    redraw();
}

void PlatformWindow::redraw()
{
    UI.frameViewer->update();
    this->update();
}



