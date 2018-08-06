#include "ScreenshotWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

ScreenshotWindow::ScreenshotWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


ScreenshotWindow::~ScreenshotWindow()
{

}


void ScreenshotWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    visu = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 1.0f, 1.0f);
    viewer->addVisualization(visu, "test");
    viewer->viewAll();
}

int ScreenshotWindow::main()
{

}



void ScreenshotWindow::on_pushButton_2_clicked()
{
    QImage image = viewer->getScreenshot();
    UI.label->setPixmap(QPixmap::fromImage(image));
}
