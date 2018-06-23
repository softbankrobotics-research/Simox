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

    VirtualRobot::VisualizationPtr visu = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/my_robot/Leg.dae");
    viewer->addVisualization("test", visu);
    viewer->viewAll();
}

int LoadMeshWindow::main()
{

}


