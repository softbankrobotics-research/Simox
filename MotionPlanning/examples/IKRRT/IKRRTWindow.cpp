
#include "IKRRTWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Model/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/Visualization/ColorMap.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "MotionPlanning/Planner/GraspIkRrt.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <QFileDialog>
#include <Eigen/Geometry>
#include <MotionPlanning/Visualization/RrtWorkspaceVisualization.h>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <QImage>
#include <QGLWidget>
#include <sstream>

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"

// need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

IKRRTWindow::IKRRTWindow(std::string& sceneFile, std::string& reachFile, std::string& rns, std::string& eef, std::string& colModel, std::string& colModelRob)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;
    this->reachFile = reachFile;
    eefName = eef;
    rnsName = rns;
    this->colModelName = colModel;
    this->colModelNameRob = colModelRob;

    playbackMode = false;

    setupUI();

    loadScene();

    loadReach();

    viewer->viewAll();

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerCB()));
    timer->start(TIMER_MS);
}


IKRRTWindow::~IKRRTWindow()
{
}


void IKRRTWindow::timerCB()
{
    float x[6];
    x[0] = (float)UI.horizontalSliderX->value();
    x[1] = (float)UI.horizontalSliderY->value();
    x[2] = (float)UI.horizontalSliderZ->value();
    x[3] = (float)UI.horizontalSliderRo->value();
    x[4] = (float)UI.horizontalSliderPi->value();
    x[5] = (float)UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        updateObject(x);
        redraw();
    }

    int maxSlider = 200;

    if (playbackMode && playCounter <= maxSlider)
    {
        if (playCounter == 0)
        {
            openEEF();
            sliderSolution(0);
            playCounter++;
        }
        else if (playCounter == maxSlider)
        {
            sliderSolution(1000);
            closeEEF();
            cout << "Stopping playback" << endl;
            playbackMode = false;
        }
        else
        {
            playCounter++;
            float pos = (float)playCounter / (float)maxSlider;
            sliderSolution((int)(pos * 1000.0f));
        }

        redraw();
        saveScreenshot();
    }
}


void IKRRTWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonIK, SIGNAL(clicked()), this, SLOT(searchIK()));

    connect(UI.checkBoxSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachableGrasps, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReachabilitySpace, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.pushButtonIKRRT, SIGNAL(clicked()), this, SLOT(planIKRRT()));
    connect(UI.pushButtonPlay, SIGNAL(clicked()), this, SLOT(playAndSave()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
    connect(UI.horizontalSliderSolution, SIGNAL(valueChanged(int)), this, SLOT(sliderSolution(int)));

    UI.checkBoxColCheckIK->setChecked(true);
    UI.checkBoxReachabilitySpaceIK->setChecked(false);

}

void IKRRTWindow::playAndSave()
{
    if (playbackMode)
    {
        playbackMode = false;
    }
    else
    {
        playCounter = 0;
        playbackMode = true;
    }
}


void IKRRTWindow::resetSceneryAll()
{
    if (rns && robot)
    {
        rns->setJointValues(startConfig);
    }
}


void IKRRTWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}



void IKRRTWindow::saveScreenshot()
{
    static int counter = 0;
    std::stringstream ss;
    ss << "renderFrame_" << counter++ << ".png";
    redraw();
    QImage i = viewer->getScreenshot();
    bool bRes = i.save(QString::fromUtf8(ss.str().c_str()), "BMP");

    if (bRes)
    {
        cout << "wrote image " << counter << endl;
    }
    else
    {
        cout << "failed writing image " << counter << endl;
    }

}

void IKRRTWindow::buildVisu()
{
    viewer->clearLayer("scene");
    showCoordSystem();

    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::Collision : ModelLink::Full;

    VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
    if (!f)
        return;

    if (robot)
    {
        VisualizationSetPtr v = robot->getVisualization(colModel);
        viewer->addVisualization("scene", v);
    }

    if (object)
    {
        ModelLink::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? ModelLink::Collision : ModelLink::Full;
        VisualizationSetPtr v = object->getVisualization(colModel2);
        viewer->addVisualization("scene", v);
    }

    if (obstacles.size() > 0)
    {
        for (size_t i = 0; i < obstacles.size(); i++)
        {
            VisualizationSetPtr v = obstacles.at(i)->getVisualization(colModel);
            viewer->addVisualization("scene", v);
        }
    }

    buildGraspSetVisu();

    buildRRTVisu();

    redraw();
}


void IKRRTWindow::quit()
{
    std::cout << "IKRRTWindow: Closing" << std::endl;
    this->close();
}

void IKRRTWindow::loadScene()
{
    graspSet.reset();
    robot.reset();
    object.reset();
    obstacles.clear();
    eef.reset();
    ScenePtr scene = SceneIO::loadScene(sceneFile);

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


    std::vector< ManipulationObjectPtr > objects = scene->getManipulationObjects();

    if (objects.size() < 1)
    {
        VR_ERROR << "Need at least 1 object" << endl;
        return;
    }

    object = objects[0];
    VR_INFO << "using first manipulation object: " << object->getName() << endl;


    obstacles = scene->getObstacles();

    if (robot && object)
    {
        eef = robot->getEndEffector(eefName);

        if (!eef)
        {
            VR_ERROR << "Need a correct EEF in robot" << endl;
            return;
        }

        graspSet = object->getGraspSet(eef);

        rns = robot->getJointSet(rnsName);

        if (!rns)
        {
            VR_ERROR << "Need a correct RNS in robot" << endl;
        }

    }

    if (rns)
    {
        rns->getJointValues(startConfig);
    }

    buildVisu();

}


void IKRRTWindow::closeEEF()
{
    if (eef)
    {
        eef->closeActors(object);
    }

    redraw();

}

void IKRRTWindow::openEEF()
{
    if (eef)
    {
        eef->openActors();
    }

    redraw();

}



void IKRRTWindow::updateObject(float x[6])
{
    if (object)
    {
        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << endl;
        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
        cout << "object " << endl;
        cout << m << endl;

    }

    redraw();

}

void IKRRTWindow::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
    buildVisu();
}

void IKRRTWindow::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
    buildVisu();
}

void IKRRTWindow::showCoordSystem()
{
    if (eef)
    {
        FramePtr tcp = eef->getTcp();

        if (!tcp)
        {
            return;
        }

        // todo
        //tcp->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }

    if (object)
    {
        // todo
        //object->showCoordinateSystem(UI.checkBoxTCP->isChecked());
    }
}

void IKRRTWindow::buildRRTVisu()
{
    viewer->clearLayer("rrt");

    if (!UI.checkBoxSolution->isChecked())
    {
        return;
    }

    if (!solution)
    {
        return;
    }

    MotionPlanning::RrtWorkspaceVisualizationPtr w(new MotionPlanning::RrtWorkspaceVisualization(robot, cspace, eef->getTcpName()));

    if (tree)
    {
        w->addTree(tree);
    }

    if (tree2)
    {
        w->addTree(tree2);
    }

    //w->addCSpacePath(solution);
    if (solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, MotionPlanning::RrtWorkspaceVisualization::eGreen);
    }

    //w->addConfiguration(startConfig,MotionPlanning::CoinRrtWorkspaceVisualization::eGreen,3.0f);
    VisualizationSetPtr wv = w->getVisualization();
    if (wv)
    {
        viewer->addVisualization("rrt", wv);
    }
}

void IKRRTWindow::buildGraspSetVisu()
{
    viewer->clearLayer("grasps");

    if (UI.checkBoxGraspSet->isChecked() && eef && graspSet && object)
    {
        VisualizationSetPtr v = graspSet->getVisualization(ModelLink::Full, eef, object->getGlobalPose());
        viewer->addVisualization("grasps", v);
    }

    // show reachable graps

    if (UI.checkBoxReachableGrasps->isChecked() && eef && graspSet && object && reachSpace)
    {
        GraspSetPtr rg = reachSpace->getReachableGrasps(graspSet, object);

        if (rg->getSize() > 0)
        {
            VisualizationSetPtr v = rg->getVisualization(ModelLink::Full, eef, object->getGlobalPose());
            viewer->addVisualization("grasps", v);
        }
    }
}


void IKRRTWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    viewer->clearLayer("reach");

    if (UI.checkBoxReachabilitySpace->checkState() == Qt::Checked)
    {
        ColorMapPtr cm(new ColorMap(ColorMap::eRed));
        VisualizationPtr v = reachSpace->getVisualization(cm, true);
        viewer->addVisualization("reach", v);

        /*
            std::shared_ptr<VirtualRobot::CoinVisualization> visualization = reachSpace->getVisualization<CoinVisualization>();
            SoNode* visualisationNode = NULL;
            if (visualization)
                visualisationNode = visualization->getCoinVisualization();

            if (visualisationNode)
                reachabilitySep->addChild(visualisationNode);
        */
    }
}

void IKRRTWindow::loadReach()
{

    if (!robot)
    {
        return;
    }

    cout << "Loading Reachability from " << reachFile << endl;
    reachSpace.reset(new Reachability(robot));

    try
    {
        reachSpace->load(reachFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while loading reach space" << endl;
        cout << e.what();
        reachSpace.reset();
        return;
    }

    reachSpace->print();

    buildVisu();
}
void IKRRTWindow::planIKRRT()
{
    openEEF();
    GenericIKSolverPtr ikSolver(new GenericIKSolver(rns));

    if (UI.checkBoxReachabilitySpaceIK->checkState() == Qt::Checked)
    {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // setup collision detection
    CDManagerPtr cdm;

    if (UI.checkBoxColCheckIK->checkState() == Qt::Checked)
    {
        LinkSetPtr colModelSet = robot->getLinkSet(colModelName);
        if (!colModelSet)
            VR_ERROR << "no col model with name " << colModelName << endl;
        LinkSetPtr colModelSet2;

        if (!colModelNameRob.empty())
        {
            colModelSet2 = robot->getLinkSet(colModelNameRob);
            if (!colModelSet2)
                VR_ERROR << "no col model with name " << colModelNameRob << endl;
        }

        if (colModelSet)
        {
            cdm.reset(new CDManager());
            cdm->addCollisionModel(object->getLinkSet());
            cdm->addCollisionModel(colModelSet);

            if (colModelSet2)
            {
                cdm->addCollisionModel(colModelSet2);
            }

            ikSolver->collisionDetection(cdm);
        }
    }
    else
    {
        cdm.reset(new CDManager());
    }

    ikSolver->setMaximumError(10.0f, 0.08f);
    ikSolver->setupJacobian(0.9f, 20);

    cspace.reset(new MotionPlanning::CSpaceSampled(robot, cdm, rns));

    GraspSetPtr graspSet = object->getGraspSet(robot->getType(), eefName);
    MotionPlanning::GraspIkRrtPtr ikRrt(new MotionPlanning::GraspIkRrt(cspace, object, ikSolver, graspSet));


    ikRrt->setStart(startConfig);
    bool planOK = ikRrt->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = ikRrt->getSolution();
        MotionPlanning::ShortcutProcessorPtr postProcessing(new MotionPlanning::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = ikRrt->getTree();
        tree2 = ikRrt->getTree2();

    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void IKRRTWindow::colModel()
{
#if 0

    if (reachSpace && graspSet && object)
    {
        Eigen::Matrix4f m = reachSpace->sampleReachablePose();
        cout << "getEntry: " << (int)reachSpace->getEntry(m) << endl;
        /*SoSeparator* sep1 = new SoSeparator;
        SoSeparator *cs = CoinVisualizationFactory::CreateCoordSystemVisualization();
        SoMatrixTransform *mt = new SoMatrixTransform;
        SbMatrix ma(reinterpret_cast<SbMat*>(m.data()));
        mt->matrix.setValue(ma);
        sep1->addChild(mt);
        sep1->addChild(cs);
        sceneSep->addChild(sep1);*/


        GraspPtr g = graspSet->getGrasp(0);
        m = g->getObjectTargetPoseGlobal(m);
        object->setGlobalPose(m);
    }

#endif
    buildVisu();
}

void IKRRTWindow::searchIK()
{
    GenericIKSolverPtr ikSolver(new GenericIKSolver(rns));

    if (UI.checkBoxReachabilitySpaceIK->checkState() == Qt::Checked)
    {
        ikSolver->setReachabilityCheck(reachSpace);
    }

    // setup collision detection
    if (UI.checkBoxColCheckIK->checkState() == Qt::Checked)
    {
        LinkSetPtr colModelSet = robot->getLinkSet(colModelName);

        if (colModelSet)
        {
            CDManagerPtr cdm(new CDManager());
            cdm->addCollisionModel(object);
            cdm->addCollisionModel(colModelSet);
            ikSolver->collisionDetection(cdm);
        }
    }

    ikSolver->setMaximumError(5.0f, 0.05f);
    ikSolver->setupJacobian(0.9f, 20);
    GraspPtr grasp = ikSolver->solve(object, IKSolver::All, 10);

    if (grasp)
    {
        VR_INFO << "IK successful..." << endl;
    }
    else
    {
        VR_INFO << "IK failed..." << endl;
    }

    redraw();
}

void IKRRTWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    MotionPlanning::CSpacePathPtr s = solution;

    if (solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    rns->setJointValues(iPos);

    redraw();
    //saveScreenshot();
}

void IKRRTWindow::redraw()
{
    UI.frameViewer->update();
    this->update();
}


