#include "VisualizationTestWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#elif Simox_USE_QT3D_VISUALIZATION
    #include "../../../Gui/Qt3D/Qt3DViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise qt3d Gui lib is not loaded since it is not referenced by us)
    SimoxGui::Qt3DViewerFactory f;
#endif

VisualizationTestWindow::VisualizationTestWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


VisualizationTestWindow::~VisualizationTestWindow()
{

}


void VisualizationTestWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    VirtualRobot::VisualizationPtr visu = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->createBox(1.0f, 1.0f, 1.0f);
    viewer->addVisualization("test", visu);
}

int VisualizationTestWindow::main()
{

}


