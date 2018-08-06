#include "VisualizationLifetimeWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

VisualizationLifetimeWindow::VisualizationLifetimeWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


VisualizationLifetimeWindow::~VisualizationLifetimeWindow()
{

}


void VisualizationLifetimeWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    visu = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 1.0f, 1.0f);
    viewer->addVisualization(visu, "test");
    viewer->viewAll();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(2000);
}

void VisualizationLifetimeWindow::update()
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
        viewer->addVisualization(visu, "test");
        toggle = true;
    }
}

int VisualizationLifetimeWindow::main()
{

}


