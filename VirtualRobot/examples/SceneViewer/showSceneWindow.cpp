
#include "showSceneWindow.h"
#include "../../../VirtualRobot/EndEffector/EndEffector.h"
#include "../../../VirtualRobot/Workspace/Reachability.h"
#include "../../../VirtualRobot/Model/ManipulationObject.h"
#include "../../../VirtualRobot/XML/ObjectIO.h"
#include "../../../VirtualRobot/Grasping/GraspSet.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

showSceneWindow::showSceneWindow(std::string& sSceneFile)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << endl;

    sceneFile = sSceneFile;

    setupUI();

    loadScene();

    viewer->viewAll();
}


showSceneWindow::~showSceneWindow()
{
}


void showSceneWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(nullptr);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectScene()));
    connect(UI.comboBoxRobot, SIGNAL(activated(int)), this, SLOT(selectRobot(int)));
    connect(UI.comboBoxObject, SIGNAL(activated(int)), this, SLOT(selectObject(int)));
    connect(UI.comboBoxRobotConfig, SIGNAL(activated(int)), this, SLOT(selectRobotConfig(int)));
    connect(UI.comboBoxTrajectory, SIGNAL(activated(int)), this, SLOT(selectTrajectory(int)));

    connect(UI.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)));

    connect(UI.pushButtonEEFClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.pushButtonEEFOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxRoot, SIGNAL(clicked()), this, SLOT(showRoot()));
}

void showSceneWindow::resetSceneryAll()
{
    updateGui();
    buildVisu();
}

void showSceneWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void showSceneWindow::colModel()
{
    buildVisu();
}


void showSceneWindow::showRoot()
{
    buildVisu();
}

void showSceneWindow::buildVisu()
{
    if (!scene)
    {
        return;
    }

    viewer->clearLayer("scene");
    ModelLink::VisualizationType visuType = ModelLink::VisualizationType::Full;
    if (UI.checkBoxColModel->isChecked())
    {
        visuType = ModelLink::VisualizationType::Collision;
    }

    VisualizationPtr visu = VisualizationFactory::getGlobalVisualizationFactory()->getVisualization(scene, visuType);
    viewer->addVisualization("scene", "scene", visu);

    if (UI.checkBoxRoot->isChecked())
    {
        std::string rootText = "ROOT";
        VisualizationNodePtr visuCoord = VisualizationFactory::getGlobalVisualizationFactory()->createCoordSystem(2.0f, &rootText);
        viewer->addVisualization("scene", "coord", visuCoord);
    }

    updateGraspVisu();
}

void showSceneWindow::updateGraspVisu()
{
    // build grasp visu
    viewer->clearLayer("grasps");

    if (UI.comboBoxGrasp->currentIndex() > 0 && currentObject && currentEEF && currentGrasp)
    {
        std::string t = currentGrasp->getName();
        VisualizationNodePtr visuCoord = VisualizationFactory::getGlobalVisualizationFactory()->createCoordSystem(2.0f, &t);
        Eigen::Matrix4f gp = currentGrasp->getTcpPoseGlobal(currentObject->getGlobalPose());
        VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(visuCoord, gp);
        viewer->addVisualization("grasps", "current-grasp", visuCoord);
    }
}

int showSceneWindow::main()
{
    viewer->start(this);
    return 0;
}


void showSceneWindow::quit()
{
    std::cout << "showSceneWindow: Closing" << std::endl;
    viewer->stop();
    this->close();
}


void showSceneWindow::sliderMoved(int pos)
{
    if (!currentTrajectory)
    {
        return;
    }

    float fpos = (float)pos / 999.0f;

    if (currentRobot)
    {
        currentRobot->setJointValues(currentTrajectory, fpos);
    }
}


void showSceneWindow::selectScene()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));
    sceneFile = std::string(fi.toLatin1());
    loadScene();
}

void showSceneWindow::loadScene()
{
    viewer->clearLayer("scene");
    currentEEF.reset();
    currentGrasp.reset();
    currentGraspSet.reset();
    currentObject.reset();
    currentRobot.reset();
    currentTrajectory.reset();
    cout << "Loading Scene from " << sceneFile << endl;

    scene.reset();

    try
    {
        scene = SceneIO::loadScene(sceneFile);
    }
    catch (VirtualRobotException& /*e*/)
    {
        cout << "Could not find valid scene in file " << sceneFile << endl;
    }

    if (!scene)
    {
        // try manip object
        try
        {

            ManipulationObjectPtr mo = ObjectIO::loadManipulationObject(sceneFile);

            if (mo)
            {
                VR_INFO << "Loaded Manipulation object:" << endl;
                mo->print();
                scene.reset(new Scene(mo->getName()));
                scene->registerManipulationObject(mo);
            }
        }
        catch (VirtualRobotException& /*e*/)
        {
            cout << "Could not find valid manipulation object in file " << sceneFile << endl;
        }
    }

    if (!scene)
    {
        // try object
        try
        {

            ObstaclePtr mo = ObjectIO::loadObstacle(sceneFile);

            if (mo)
            {
                VR_INFO << "Loaded obstacle:" << endl;
                mo->print();
                scene.reset(new Scene(mo->getName()));
                scene->registerObstacle(mo);
            }
        }
        catch (VirtualRobotException& /*e*/)
        {
            cout << "Could not find valid obstacle in file " << sceneFile << endl;
        }
    }

    if (!scene)
    {
        cout << " ERROR while creating scene" << endl;
        return;
    }

    updateGui();
    buildVisu();
    viewer->viewAll();
}

void showSceneWindow::selectRobot(int nr)
{
    UI.comboBoxRobotConfig->clear();
    UI.comboBoxTrajectory->clear();
    UI.comboBoxEEF->clear();
    currentRobot.reset();

    if (nr < 0 || nr >= UI.comboBoxRobot->count() || !scene)
    {
        return;
    }

    std::string robName(UI.comboBoxRobot->currentText().toLatin1());
    currentRobot = scene->getRobot(robName);

    if (!currentRobot)
    {
        return;
    }

    std::vector<VirtualRobot::RobotConfigPtr> roc = scene->getRobotConfigs(currentRobot);

    for (size_t i = 0; i < roc.size(); i++)
    {
        QString rn = roc[i]->getName().c_str();
        UI.comboBoxRobotConfig->addItem(rn);
    }

    if (roc.size() > 0)
    {
        UI.comboBoxRobotConfig->setCurrentIndex(0);
    }

    std::vector<VirtualRobot::TrajectoryPtr> tr = scene->getTrajectories(currentRobot->getName());

    for (size_t i = 0; i < tr.size(); i++)
    {
        QString rn = tr[i]->getName().c_str();
        UI.comboBoxTrajectory->addItem(rn);
    }

    if (tr.size() > 0)
    {
        UI.comboBoxTrajectory->setCurrentIndex(0);
    }


    std::vector<VirtualRobot::EndEffectorPtr> eefs = currentRobot->getEndEffectors();

    for (size_t i = 0; i < eefs.size(); i++)
    {
        QString rn = eefs[i]->getName().c_str();
        UI.comboBoxEEF->addItem(rn);
    }

    selectEEF(0);


    selectRobotConfig(0);
    selectTrajectory(0);
}

void showSceneWindow::selectRobotConfig(int nr)
{
    if (nr < 0 || nr >= UI.comboBoxRobotConfig->count() || !scene || !currentRobot)
    {
        return;
    }

    std::string s(UI.comboBoxRobotConfig->currentText().toLatin1());
    VirtualRobot::RobotConfigPtr rc = scene->getRobotConfig(currentRobot->getName(), s);

    if (!rc)
    {
        return;
    }

    currentRobot->setJointValues(rc);
}

void showSceneWindow::selectTrajectory(int nr)
{
    UI.horizontalSlider->setSliderPosition(0);

    if (nr < 0 || nr >= UI.comboBoxTrajectory->count() || !scene)
    {
        currentTrajectory.reset();
        UI.horizontalSlider->setEnabled(false);
        return;
    }

    UI.horizontalSlider->setEnabled(true);
    std::string s(UI.comboBoxTrajectory->currentText().toLatin1());
    currentTrajectory = scene->getTrajectory(s);
    sliderMoved(0);
}

void showSceneWindow::selectEEF(int nr)
{
    if (nr < 0 || nr >= UI.comboBoxEEF->count() || !currentRobot)
    {
        currentEEF.reset();
        return;
    }

    std::string eefStr(UI.comboBoxEEF->currentText().toLatin1());
    currentEEF = currentRobot->getEndEffector(eefStr);
    updateGrasps();
}

void showSceneWindow::selectObject(int nr)
{
    if (!scene || nr < 0 || nr >= UI.comboBoxObject->count())
    {
        return;
    }

    std::string ob(UI.comboBoxObject->currentText().toLatin1());
    currentObject.reset();

    if (scene->hasManipulationObject(ob))
    {
        VirtualRobot::ManipulationObjectPtr mo = scene->getManipulationObject(ob);
        currentObject = std::dynamic_pointer_cast<Model>(mo);
    }

    updateGrasps();
}

void showSceneWindow::selectGrasp(int nr)
{
    currentGrasp.reset();

    if (nr <= 0 || nr >= UI.comboBoxGrasp->count() || !currentGraspSet)
    {
        return;
    }

    std::string grStr(UI.comboBoxGrasp->currentText().toLatin1());

    if (currentGraspSet->hasGrasp(grStr))
    {
        currentGrasp = currentGraspSet->getGrasp(grStr);
    }

    updateGraspVisu();
}

void showSceneWindow::updateGui()
{
    UI.comboBoxObject->clear();
    UI.comboBoxRobot->clear();
    UI.comboBoxRobotConfig->clear();
    UI.comboBoxTrajectory->clear();
    UI.comboBoxEEF->clear();


    currentRobot.reset();

    if (!scene)
    {
        return;
    }

    std::vector<VirtualRobot::RobotPtr> robs = scene->getRobots();

    for (size_t i = 0; i < robs.size(); i++)
    {
        QString rn = robs[i]->getName().c_str();
        UI.comboBoxRobot->addItem(rn);
    }

    std::vector<VirtualRobot::ManipulationObjectPtr> mos = scene->getManipulationObjects();

    for (size_t i = 0; i < mos.size(); i++)
    {
        QString mn = mos[i]->getName().c_str();
        UI.comboBoxObject->addItem(mn);
    }

    std::vector<VirtualRobot::ObstaclePtr> obs = scene->getObstacles();

    for (size_t i = 0; i < obs.size(); i++)
    {
        QString on = obs[i]->getName().c_str();
        UI.comboBoxObject->addItem(on);
    }

    if (robs.size() > 0)
    {
        UI.comboBoxRobot->setCurrentIndex(0);
        selectRobot(0);
    }

    if (obs.size() > 0)
    {
        UI.comboBoxRobot->setCurrentIndex(0);
        selectObject(0);
    }
}

void showSceneWindow::updateGrasps()
{
    currentGraspSet.reset();
    UI.comboBoxGrasp->clear();
    QString t("-");
    UI.comboBoxGrasp->addItem(t);
    VirtualRobot::ManipulationObjectPtr mo = std::dynamic_pointer_cast<ManipulationObject>(currentObject);

    if (mo && currentEEF)
    {
        currentGraspSet = mo->getGraspSet(currentEEF);

        if (currentGraspSet)
            for (unsigned int i = 0; i < currentGraspSet->getSize(); i++)
            {
                t = currentGraspSet->getGrasp(i)->getName().c_str();
                UI.comboBoxGrasp->addItem(t);
            }
    }

    UI.comboBoxGrasp->setCurrentIndex(0);
    selectGrasp(0);
}

void showSceneWindow::closeHand()
{
    if (!currentEEF)
    {
        return;
    }

    VirtualRobot::ModelPtr so;

    if (UI.comboBoxObject->currentIndex() >= 0)
    {
        if (UI.comboBoxObject->currentIndex() < (int)scene->getManipulationObjects().size())
        {
            std::string s(UI.comboBoxObject->currentText().toLatin1());
            so = scene->getManipulationObject(s);
        }
        else
        {
            std::string s(UI.comboBoxObject->currentText().toLatin1());
            so = scene->getObstacle(s);
        }
    }

    currentEEF->closeActors(so);
}

void showSceneWindow::openHand()
{
    if (!currentEEF)
    {
        return;
    }

    currentEEF->openActors();
}

