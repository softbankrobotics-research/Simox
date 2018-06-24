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
    viewer->addVisualization("axis", visuX);

    VirtualRobot::VisualizationPtr visuXaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 0.002f, 0.002f);
    visuXaxis->setGlobalPose(xmataxis);
    visuXaxis->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    viewer->addVisualization("axis", visuXaxis);

    VirtualRobot::VisualizationPtr visuY = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.02f, 0.02f, 0.02f);
    visuY->setGlobalPose(ymat);
    visuY->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    viewer->addVisualization("axis", visuY);

    VirtualRobot::VisualizationPtr visuYaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.002f, 1.0f, 0.002f);
    visuYaxis->setGlobalPose(ymataxis);
    visuYaxis->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    viewer->addVisualization("axis", visuYaxis);

    VirtualRobot::VisualizationPtr visuZ = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.02f, 0.02f, 0.02f);
    visuZ->setGlobalPose(zmat);
    visuZ->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    viewer->addVisualization("axis", visuZ);

    VirtualRobot::VisualizationPtr visuZaxis = VirtualRobot::VisualizationFactory::getInstance()->createBox(0.002f, 0.002f, 1.0f);
    visuZaxis->setGlobalPose(zmataxis);
    visuZaxis->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    viewer->addVisualization("axis", visuZaxis);



    VirtualRobot::VisualizationPtr visu1 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.stl");
    viewer->addVisualization("test", visu1);

    VirtualRobot::VisualizationPtr visu2 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.dae");
    viewer->addVisualization("test", visu2);

    /*VirtualRobot::VisualizationPtr visu3 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.obj");
    viewer->addVisualization("test", visu3);

    VirtualRobot::VisualizationPtr visu4 = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/ArmL_Elb1_joint_visu.x3d");
    viewer->addVisualization("test", visu4);*/

    //viewer->viewAll();
}

int LoadMeshWindow::main()
{

}


