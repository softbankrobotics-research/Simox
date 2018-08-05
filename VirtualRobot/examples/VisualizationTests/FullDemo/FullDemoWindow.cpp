#include "FullDemoWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

FullDemoWindow::FullDemoWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


FullDemoWindow::~FullDemoWindow()
{

}


void FullDemoWindow::setupUI()
{
    UI.setupUi(this);

    UI.tabWidget->setCurrentIndex(0);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer1 = viewerFactory->createViewer(UI.frameViewer1);
    viewer2 = viewerFactory->createViewer(UI.frameViewer2);

    VirtualRobot::VisualizationPtr cube = VirtualRobot::VisualizationFactory::getInstance()->createBox(1000.0f, 1000.0f, 1000.0f);
    viewer1->addVisualization("test", cube);
    viewer1->viewAll();

    VirtualRobot::VisualizationPtr line = VirtualRobot::VisualizationFactory::getInstance()->createCircle(500.0f, 0.5f, 1.0f);
    viewer2->addVisualization("test", line);
    viewer2->viewAll();

    line->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
}

int FullDemoWindow::main()
{

}


