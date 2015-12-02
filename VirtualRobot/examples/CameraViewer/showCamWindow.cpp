
#include "showCamWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoUnits.h>

#include <sstream>

//#include <GL/gl.h>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showCamWindow::showCamWindow(std::string& sRobotFilename, std::string& cam1Name, std::string& cam2Name, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);
    cam1Renderer = NULL;
    cam2Renderer = NULL;
    cam1Buffer = NULL;
    cam2Buffer = NULL;
    cam1DepthBuffer = NULL;
    cam2DepthBuffer = NULL;

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    m_sRobotFilename = sRobotFilename;
    this->cam1Name = cam1Name;
    this->cam2Name = cam2Name;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    /*SoUnits *u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    sceneSep->addChild(u);*/
    robotSep = new SoSeparator;
    extraSep = new SoSeparator;
    sceneSep->addChild(extraSep);
    //add objects
    Eigen::Matrix4f m;
    m.setIdentity();
    m(2, 3) = 1500.0f;

    m(1, 3) = 1500.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(400.0f));
    visuObjects.back()->setGlobalPose(m);
    extraSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(visuObjects.back(), VirtualRobot::SceneObject::Full));

    m(0, 3) = 700.0f;
    m(1, 3) = 900.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(300.0f));
    visuObjects.back()->setGlobalPose(m);
    extraSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(visuObjects.back(), VirtualRobot::SceneObject::Full));

    m(0, 3) = 0.0f;
    m(1, 3) = 2000.0f;
    m(2, 3) = 2000.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(200.0f));
    visuObjects.back()->setGlobalPose(m);
    extraSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(visuObjects.back(), VirtualRobot::SceneObject::Full));

    m(0, 3) = 500.0f;
    m(1, 3) = 1500.0f;
    m(2, 3) = 2000.0f;
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(200.0f));
    visuObjects.back()->setGlobalPose(m);
    extraSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(visuObjects.back(), VirtualRobot::SceneObject::Full));

    /*SoShapeHints * shapeHints = new SoShapeHints;
    shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    shapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    sceneSep->addChild(shapeHints);*/
    /*SoLightModel * lightModel = new SoLightModel;
    lightModel->model = SoLightModel::BASE_COLOR;
    sceneSep->addChild(lightModel);*/

    sceneSep->addChild(robotSep);

    cam1VoxelSep = new SoSeparator;
    cam1VoxelSep->ref();
    sceneSep->addChild(cam1VoxelSep);

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showCamWindow::~showCamWindow()
{
    robot.reset();
    sceneSep->unref();
    cam1VoxelSep->unref();

    UI.cam1->setScene(NULL);
    visuObjects.emplace_back(VirtualRobot::Obstacle::createSphere(400.0f));
    UI.cam2->setScene(NULL);

    delete [] cam1Buffer;
    delete [] cam2Buffer;
    delete [] cam1DepthBuffer;
    delete [] cam2DepthBuffer;
}



void showCamWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    //#ifndef _DEBUG
    viewer->setAntialiasing(true, 4);
    //#endif
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

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
//    connect(UI.checkBoxShowDepthVoxel, SIGNAL(clicked()), this, SLOT(renderCam()));
}

QString showCamWindow::formatString(const char* s, float f)
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

void showCamWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<float> jv(allRobotNodes.size(), 0.0f);
    robot->setJointValues(allRobotNodes, jv);

    selectJoint(UI.comboBoxJoint->currentIndex());
}

void showCamWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();
    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = false;
    //bool sensors = UI.checkBoxRobotSensors->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = useColModel ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    selectJoint(UI.comboBoxJoint->currentIndex());
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

int showCamWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void showCamWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
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
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
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
        currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
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
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }

    cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

    if (visualization)
    {
        robot->highlight(visualization, false);
        currentRobotNode->highlight(visualization, true);
    }

}

void showCamWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
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
    std::string s = m_sRobotFilename = std::string(fi.toAscii());

    if (!s.empty())
    {
        m_sRobotFilename = s;
        loadRobot();
    }
}

void showCamWindow::loadRobot()
{
    robotSep->removeAllChildren();
    cout << "Loading Robot from " << m_sRobotFilename << endl;
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        QFileInfo fileInfo(m_sRobotFilename.c_str());
        std::string suffix(fileInfo.suffix().toAscii());
        RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

        if (!importer)
        {
            cout << " ERROR while grabbing importer" << endl;
            return;
        }

        robot = importer->loadFromFile(m_sRobotFilename, RobotIO::eFull);


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
    cam1Renderer = NULL;
    cam2Renderer = NULL;
    delete []cam1Buffer;
    cam1Buffer = NULL;
    delete []cam2Buffer;
    cam2Buffer = NULL;
    delete [] cam1DepthBuffer;
    cam1DepthBuffer = NULL;
    delete [] cam2DepthBuffer;
    cam2DepthBuffer = NULL;

    if (!robot)
    {
        return;
    }

    if (robot->hasRobotNode(cam1Name))
    {
        cam1 = robot->getRobotNode(cam1Name);
    }

    if (robot->hasRobotNode(cam2Name))
    {
        cam2 = robot->getRobotNode(cam2Name);
    }

    if (cam1)
    {
        cam1Renderer = CoinVisualizationFactory::createOffscreenRenderer(UI.cam1->size().width(), UI.cam1->size().height());
        cam1Buffer = new unsigned char [UI.cam1->size().width() * UI.cam1->size().height() * 3];

        cam1DepthBuffer = new float [UI.cam1->size().width() * UI.cam1->size().height()];

        userdata1.buffer = cam1DepthBuffer;
        userdata1.w = UI.cam1->size().width();
        userdata1.h = UI.cam1->size().height();

        cam1Renderer->getGLRenderAction()->setPassCallback(getDepthImage, &userdata1);
        cam1Renderer->getGLRenderAction()->setNumPasses(2);
    }

    if (cam2)
    {
        cam2Renderer = CoinVisualizationFactory::createOffscreenRenderer(UI.cam2->size().width(), UI.cam2->size().height());
        cam2Buffer = new unsigned char [UI.cam2->size().width() * UI.cam2->size().height() * 3];

        cam2DepthBuffer = new float [UI.cam2->size().width() * UI.cam2->size().height()];

        userdata2.buffer = cam2DepthBuffer;
        userdata2.w = UI.cam2->size().width();
        userdata2.h = UI.cam2->size().height();

        cam2Renderer->getGLRenderAction()->setPassCallback(getDepthImage, &userdata2);
        cam2Renderer->getGLRenderAction()->setNumPasses(2);
    }

    renderCam();
}

//Zeye is negative
float convertDepthToZEye(float depth, float zNear, float zFar)
{
    return -zFar*zNear/(zFar + depth*(zNear - zFar));
}

void showCamWindow::renderCam()
{
    const float zNear = 10;
    const float zFar = 100000;
    const float fov = M_PI / 4;
    const float maxZCut = UI.doubleSpinBoxDepthLinClip->value();
    const float voxelSize= 10.f;
    float focal;

    if (cam1 && cam1Renderer)
    {

        //always clean up voxel (we don't want to include the voxels in the depth image)
        cam1VoxelSep->removeAllChildren();
        voxelObjects.clear();

        //render image
        std::tie(std::ignore, focal) = CoinVisualizationFactory::renderOffscreenAndGetFocalLength(cam1Renderer, cam1, sceneSep, &cam1Buffer, zNear, zFar, fov);
        if(UI.checkBoxDepthCam1->isChecked())
        {
            //transform
            for(std::size_t index = 0; index < UI.cam1->size().width()*UI.cam1->size().height(); ++index)
            {
                const float distance = -convertDepthToZEye(cam1DepthBuffer[index],zNear,zFar);
                const unsigned char value = (distance>=maxZCut)?255:distance/maxZCut*255.f;

                cam1Buffer[3*index] = value;
                cam1Buffer[3*index+1] = value;
                cam1Buffer[3*index+2] = value;
            }
        }
        QImage img1(cam1Buffer, UI.cam1->size().width(), UI.cam1->size().height(), QImage::Format_RGB888);
        //UI.cam1->setPixmap(QPixmap::fromImage(img1));

        QGraphicsScene* scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img1.mirrored(false, true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene* oldScene = UI.cam1->scene();
        UI.cam1->setScene(scene);
        delete oldScene;
    }

    if (cam2 && cam2Renderer)
    {
        CoinVisualizationFactory::renderOffscreen(cam2Renderer, cam2, sceneSep, &cam2Buffer, zNear, zFar, fov);
        if(UI.checkBoxDepthCam2->isChecked())
        {
            //transform
            for(std::size_t index = 0; index < UI.cam2->size().width()*UI.cam2->size().height(); ++index)
            {
                const unsigned char value = cam2DepthBuffer[index]*255* UI.doubleSpinBoxNonLinFactor->value();
                cam2Buffer[3*index] = value;
                cam2Buffer[3*index+1] = value;
                cam2Buffer[3*index+2] = value;
            }
        }
        QImage img2(cam2Buffer, UI.cam2->size().width(), UI.cam2->size().height(), QImage::Format_RGB888);
        //UI.cam2->setPixmap(QPixmap::fromImage(img2));

        QGraphicsScene* scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img2.mirrored(false, true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene* oldScene = UI.cam2->scene();
        UI.cam2->setScene(scene);
        delete oldScene;
    }
//    //draw voxel
//    if (cam1 && cam1Renderer && UI.checkBoxShowDepthVoxel->isChecked())
//    {
//        //add transform and canvas separator
//        Eigen::Matrix4f cam1Transform = cam1->getGlobalPose().transpose().inverse();
//        SoMatrixTransform* cam1TransformSep = new SoMatrixTransform();

//        cam1TransformSep->matrix.setValue(
//            cam1Transform(0,0), cam1Transform(0,1), cam1Transform(0,2), cam1Transform(0,3),
//            cam1Transform(1,0), cam1Transform(1,1), cam1Transform(1,2), cam1Transform(1,3),
//            cam1Transform(2,0), cam1Transform(2,1), cam1Transform(2,2), cam1Transform(2,3),
//            0,0,0, cam1Transform(3,3)
//        );

//        cam1VoxelSep->addChild(cam1TransformSep);

//        SoSeparator* voxelCanvasSep = new SoSeparator();
//        cam1VoxelSep->addChild(voxelCanvasSep);

//        //calculate parameters
//        //image's half height (aka height meassured from focal point to top)
//        const float height = tan(fov)*focal;
//        //#pixel from focal point to top/side of the image
//        const auto pCntX = UI.cam1->size().width() /2;
//        const auto pCntY = UI.cam1->size().height() /2;
//        const float pixelHeight = height / pCntY;

//        const Eigen::Vector3f focalAxis(focal, 0, 0);
//        const Eigen::Vector3f e2(0,1,0);
//        const Eigen::Vector3f e3(0,0,1);

//        Eigen::Matrix4f m;
//        m.setIdentity();

//        //draw voxel
//        for(int x = 0; x < UI.cam1->size().width(); x+=10)
//        {
//            for(int y = 0; y < UI.cam1->size().height(); y+=10)
//            {
//                //pixel coords relative to focal point
//                const int xI = (x - pCntX);
//                const int yI = (y - pCntY);

//                const unsigned int index = x + y * UI.cam1->size().width();
//                const float distance = -convertDepthToZEye(cam1DepthBuffer[index],zNear,zFar);
//                const unsigned char value = (distance>=maxZCut)?255:distance/maxZCut*255.f;
//                const float restoredDistance = value/255.f*maxZCut;

//                 Eigen::Vector3f pixelPos = focalAxis+ pixelHeight*(xI*e2+yI*e3);

//                pixelPos.normalize();
//                const Eigen::Vector3f position = pixelPos*restoredDistance;

//                //rotate+mirror
//                const float scale = 0.8;
//                m(0, 3) = position(0) * scale - cam1Transform(3,2);
//                m(1, 3) = position(2) * scale - cam1Transform(3,0);
//                m(2, 3) = position(1) * scale - cam1Transform(3,1);

//                voxelObjects.emplace_back(VirtualRobot::Obstacle::createBox(voxelSize,voxelSize,voxelSize,VisualizationFactory::Color::Green()));
//                voxelObjects.back()->setGlobalPose(m);
//                voxelCanvasSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(voxelObjects.back(), VirtualRobot::SceneObject::Full));

//            }
//        }
//    }
}

void showCamWindow::updatRobotInfo()
{
    if (!robot)
    {
        return;
    }

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);

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

void showCamWindow::getDepthImage(void *userdata)
{
    DepthRenderData* ud = reinterpret_cast<DepthRenderData*>(userdata);
    glReadPixels(0, 0, ud->w, ud->h, GL_DEPTH_COMPONENT, GL_FLOAT, ud->buffer);
}

