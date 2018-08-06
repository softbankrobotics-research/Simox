#include "LayersWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

LayersWindow::LayersWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


LayersWindow::~LayersWindow()
{

}


void LayersWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    VirtualRobot::VisualizationPtr visu = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 1.0f, 1.0f);
    viewer->addVisualization(visu, "test");
    viewer->viewAll();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(2000);
}

void LayersWindow::update()
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
        VirtualRobot::VisualizationPtr visu = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 1.0f, 1.0f);
        viewer->addVisualization(visu, "test");
        toggle = true;
    }
}

int LayersWindow::main()
{

}


