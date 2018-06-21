#include "ViewAllWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

ViewAllWindow::ViewAllWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


ViewAllWindow::~ViewAllWindow()
{

}


void ViewAllWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    visu = VirtualRobot::VisualizationFactory::getInstance()->createBox(5000.0f, 5000.0f, 5000.0f);
    viewer->addVisualization("test", visu);
    viewer->viewAll();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(2000);
}

void ViewAllWindow::update()
{
    if(toggle)
    {
        std::cout << "clear" << std::endl;
        viewer->clearLayer("test");
        toggle = false;
    }
    else
    {
        std::cout << "add" << std::endl;
        viewer->addVisualization("test", visu);
        toggle = true;
    }
}

int ViewAllWindow::main()
{

}


