#include "LoadMeshWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

LoadMeshWindow::LoadMeshWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


LoadMeshWindow::~LoadMeshWindow()
{

}


void LoadMeshWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    Eigen::Matrix4f xmat, ymat, zmat, xmataxis, ymataxis, zmataxis;

    xmat <<
         1.0f, 0.0f, 0.0f, 1000.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    xmataxis <<
         1.0f, 0.0f, 0.0f, 500.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    ymat <<
         1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 1000.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    ymataxis <<
         1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 500.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    zmat <<
         1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 1000.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    zmataxis <<
         1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 500.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    VirtualRobot::VisualizationPtr visuX = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.02f, 0.02f, 0.02f);
    visuX->setGlobalPose(xmat);
    visuX->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    visuX->scale(1000.0f);
    viewer->addVisualization(visuX, "axis");

    VirtualRobot::VisualizationPtr visuXaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 0.002f, 0.002f);
    visuXaxis->setGlobalPose(xmataxis);
    visuXaxis->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    visuXaxis->scale(1000.0f);
    viewer->addVisualization(visuXaxis, "axis");

    VirtualRobot::VisualizationPtr visuY = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.02f, 0.02f, 0.02f);
    visuY->setGlobalPose(ymat);
    visuY->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    visuY->scale(1000.0f);
    viewer->addVisualization(visuY, "axis");

    VirtualRobot::VisualizationPtr visuYaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.002f, 1.0f, 0.002f);
    visuYaxis->setGlobalPose(ymataxis);
    visuYaxis->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    visuYaxis->scale(1000.0f);
    viewer->addVisualization(visuYaxis, "axis");

    VirtualRobot::VisualizationPtr visuZ = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.02f, 0.02f, 0.02f);
    visuZ->setGlobalPose(zmat);
    visuZ->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    visuZ->scale(1000.0f);
    viewer->addVisualization(visuZ, "axis");

    VirtualRobot::VisualizationPtr visuZaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.002f, 0.002f, 1.0f);
    visuZaxis->setGlobalPose(zmataxis);
    visuZaxis->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    visuZaxis->scale(1000.0f);
    viewer->addVisualization(visuZaxis, "axis");



    VirtualRobot::VisualizationPtr visu1 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.stl");
    visu1->scale(1000.0f);
    viewer->addVisualization(visu1, "test");

    VirtualRobot::VisualizationPtr visu2 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.dae");
    visu2->scale(1000.0f);
    viewer->addVisualization(visu2, "test");

    /*VirtualRobot::VisualizationPtr visu3 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.obj");
    viewer->addVisualization(visu3, "test");

    VirtualRobot::VisualizationPtr visu4 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.x3d");
    viewer->addVisualization(visu4, "test");*/

    //viewer->viewAll();
}

int LoadMeshWindow::main()
{

}


