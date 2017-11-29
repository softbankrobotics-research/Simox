#include "showCamWindow.h"
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/OffscreenRenderer.h>

#include <QFileDialog>
#include <QGraphicsScene>

#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showCamWindow::showCamWindow(std::string& sRobotFilename, std::string& cam1Name, std::string& cam2Name)
    : QMainWindow(nullptr),
      obstacleVisu(new VirtualRobot::VisualizationGroup),
      robotVisu(VisualizationFactory::getGlobalVisualizationFactory()->createVisualization())
{
    VR_INFO << " start " << endl;
    setupUI();

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    robotFilename = sRobotFilename;
    this->cam1Name = cam1Name;
    this->cam2Name = cam2Name;


    //add objects
    Eigen::Matrix4f m;
    m.setIdentity();
    m(2, 3) = 1500.0f;

    m(1, 3) = 1500.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(400.0f));
    visuObjects.back()->setGlobalPose(m);
    obstacleVisu->addVisualization(visuObjects.back()->getVisualization(VirtualRobot::ModelLink::VisualizationType::Full));

    m(0, 3) = 700.0f;
    m(1, 3) = 900.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(300.0f));
    visuObjects.back()->setGlobalPose(m);
    obstacleVisu->addVisualization(visuObjects.back()->getVisualization(VirtualRobot::ModelLink::VisualizationType::Full));

    m(0, 3) = 0.0f;
    m(1, 3) = 2000.0f;
    m(2, 3) = 2000.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(200.0f));
    visuObjects.back()->setGlobalPose(m);
    obstacleVisu->addVisualization(visuObjects.back()->getVisualization(VirtualRobot::ModelLink::VisualizationType::Full));

    m(0, 3) = 500.0f;
    m(1, 3) = 1500.0f;
    m(2, 3) = 2000.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(200.0f));
    visuObjects.back()->setGlobalPose(m);
    obstacleVisu->addVisualization(visuObjects.back()->getVisualization(VirtualRobot::ModelLink::VisualizationType::Full));

    viewer->addVisualizations("obstacles", obstacleVisu);

    loadRobot();

    viewer->viewAll();
}

showCamWindow::~showCamWindow()
{
    robot.reset();

    UI.cam1->setScene(nullptr);
    UI.cam2->setScene(nullptr);
}

void showCamWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

    connect(UI.checkBoxDepthCam1, SIGNAL(clicked()), this, SLOT(renderCam()));
    connect(UI.checkBoxDepthCam2, SIGNAL(clicked()), this, SLOT(renderCam()));

    connect(UI.horizontalSliderRobotY, SIGNAL(valueChanged(int)), this, SLOT(updateRobotY(int)));
    connect(UI.doubleSpinBoxNonLinFactor, SIGNAL(valueChanged(double)), this, SLOT(renderCam()));
    connect(UI.doubleSpinBoxDepthLinClip, SIGNAL(valueChanged(double)), this, SLOT(renderCam()));
    connect(UI.checkBoxShowDepthVoxel, SIGNAL(clicked()), this, SLOT(renderCam()));
}

void showCamWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    for (auto & n:allRobotNodes)
    {
        n->setJointValueNoUpdate(0.0f);
    }
    robot->applyJointValues();

    selectJoint(UI.comboBoxJoint->currentIndex());
}

void showCamWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = false;
    //bool sensors = UI.checkBoxRobotSensors->checkState() == Qt::Checked;
    ModelLink::VisualizationType colModel = useColModel ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    viewer->clearLayer("robot");
    robotVisu = robot->getVisualization(colModel);
    viewer->addVisualization("robot", robotVisu);
}

void showCamWindow::showRobot()
{
    //m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void showCamWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void showCamWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
}

void showCamWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
    }
}

void showCamWindow::updateRobotY(int pos)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(1,3)=pos;
    if (robot)
        robot->setGlobalPose(pose);
    renderCam();
}

void showCamWindow::updateRNSBox()
{
    UI.comboBoxRobotNodeSet->clear();
    UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }
}

void showCamWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentRobotNodes = allRobotNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getJoints();
        /*cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << endl;
        if (visualization)
        {

            robot->highlight(visualization,false);
            currentRobotNodeSet->highlight(visualization,true);
        }*/

    }

    updateJointBox();
    selectJoint(0);
}

void showCamWindow::selectJoint(int nr)
{
    if (currentRobotNode)
    {
        //currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    //currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLow();
    float ma = currentRobotNode->getJointLimitHigh();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = (int)((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        //UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }

    // TODO
    //cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

}

void showCamWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLow() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHigh() - currentRobotNodes[nr]->getJointLimitLow());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    renderCam();
}

void showCamWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr(filter.c_str()));
    std::string s = robotFilename = std::string(fi.toLatin1());

    if (!s.empty())
    {
        robotFilename = s;
        loadRobot();
    }
}

void showCamWindow::loadRobot()
{
    cout << "Loading Robot from " << robotFilename << endl;
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        robot = ModelIO::loadModel(robotFilename, ModelIO::eFull);
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

    updateCameras();
    updatRobotInfo();
}

void showCamWindow::updateCameras()
{
    cam1.reset();
    cam2.reset();

    if (!robot)
    {
        return;
    }

    if (robot->hasFrame(cam1Name))
    {
        cam1 = robot->getFrame(cam1Name);
    }

    if (robot->hasFrame(cam2Name))
    {
        cam2 = robot->getFrame(cam2Name);
    }

    if (cam1)
    {
        const auto pixelCount = UI.cam1->size().width() * UI.cam1->size().height();
        cam1RGBBuffer.resize(pixelCount*3);
        cam1DepthBuffer.resize(pixelCount);
        cam1PointCloud.resize(pixelCount);
    }

    if (cam2)
    {
        const auto pixelCount = UI.cam2->size().width() * UI.cam2->size().height();
        cam2RGBBuffer.resize(pixelCount*3);
        cam2DepthBuffer.resize(pixelCount);
        cam2PointCloud.resize(pixelCount);
    }

    renderCam();
}

void showCamWindow::renderCam()
{
    const float zNear = 10;
    const float zFar = 100000;
    const float fov = M_PI / 4;
    const float maxZCut = UI.doubleSpinBoxDepthLinClip->value();

    std::vector<VisualizationPtr> scene = obstacleVisu->getVisualizations();
    scene.push_back(robotVisu);

    if (cam1)
    {
        voxelObjects.clear();

        OffscreenRenderer::getGlobalOffscreenRenderer()->renderOffscreen(cam1->getGlobalPose(),
                                                                         scene,
                                                                         UI.cam1->size().width(), UI.cam1->size().height(),
                                                                         true, cam1RGBBuffer,
                                                                         true, cam1DepthBuffer,
                                                                         true, cam1PointCloud,
                                                                         zNear, zFar, fov);

        if(UI.checkBoxDepthCam1->isChecked())
        {
            //transform
            for(std::size_t index = 0; index < static_cast<std::size_t>(UI.cam1->size().width()*UI.cam1->size().height()); ++index)
            {
                const float distance = cam1DepthBuffer.at(index);
                const unsigned char value = (distance>=maxZCut)?255:distance/maxZCut*255.f;

                cam1RGBBuffer.at(3 * index    ) = value;
                cam1RGBBuffer.at(3 * index + 1) = value;
                cam1RGBBuffer.at(3 * index + 2) = value;
            }
        }
        QImage img1(cam1RGBBuffer.data(), UI.cam1->size().width(), UI.cam1->size().height(), QImage::Format_RGB888);
        //UI.cam1->setPixmap(QPixmap::fromImage(img1));

        QGraphicsScene* scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img1.mirrored(false, true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene* oldScene = UI.cam1->scene();
        UI.cam1->setScene(scene);
        delete oldScene;
    }

    if (cam2)
    {
        OffscreenRenderer::getGlobalOffscreenRenderer()->renderOffscreen(cam2->getGlobalPose(),
                                                                         scene,
                                                                         UI.cam2->size().width(), UI.cam2->size().height(),
                                                                         true, cam2RGBBuffer,
                                                                         true, cam2DepthBuffer,
                                                                         true, cam2PointCloud,
                                                                         zNear, zFar, fov);
        if(UI.checkBoxDepthCam2->isChecked())
        {
            //transform
            for(std::size_t index = 0; index < static_cast<std::size_t>(UI.cam2->size().width()*UI.cam2->size().height()); ++index)
            {
                const float distance = cam2DepthBuffer.at(index);
                const unsigned char value = (distance>=maxZCut)?255:distance/maxZCut*255.f;

                cam2RGBBuffer.at(3 * index    ) = value;
                cam2RGBBuffer.at(3 * index + 1) = value;
                cam2RGBBuffer.at(3 * index + 2) = value;
            }
        }
        QImage img2(cam2RGBBuffer.data(), UI.cam2->size().width(), UI.cam2->size().height(), QImage::Format_RGB888);
        //UI.cam2->setPixmap(QPixmap::fromImage(img2));

        QGraphicsScene* scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img2.mirrored(false, true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene* oldScene = UI.cam2->scene();
        UI.cam2->setScene(scene);
        delete oldScene;
    }
    //draw voxel
    viewer->clearLayer("pcl");
    if (cam1 && UI.checkBoxShowDepthVoxel->isChecked())
    {
//        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
//        rotation(0,0) = 0;
//        rotation(0,1) = -1;

//        rotation(1,0) = 1;
//        rotation(1,1) = 0;
        Eigen::Matrix4f cam1Transform = cam1->getGlobalPose();// * rotation;


        for (auto& p : cam1PointCloud)
        {
            p = (cam1Transform* Eigen::Vector4f(p(0), p(1), p(2), 1.f)).block<3, 1>(0, 0);
        }
        cam1pclVisu = VisualizationFactory::getGlobalVisualizationFactory()->createPointCloud(cam1PointCloud, 4.f);
        viewer->addVisualization("pcl", cam1pclVisu);
    }
}

void showCamWindow::updatRobotInfo()
{
    if (!robot)
    {
        return;
    }

    // get nodes
    allRobotNodes = robot->getJoints();
    robotNodeSets = robot->getJointSets();

    updateRNSBox();
    selectRNS(0);

    if (allRobotNodes.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }

    // build visualization
    rebuildVisualization();

    viewer->viewAll();
}

