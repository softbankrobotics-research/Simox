
#ifndef __ShowVISU_WINDOW_H_
#define __ShowVISU_WINDOW_H_

#include "../../../VirtualRobot/Visualization/VisualizationFactory.h"
#include "../../../Gui/ViewerInterface.h"
#include "../../../Gui/ViewerFactory.h"

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <string.h>
#include <vector>

#include "ui_VisualizationTest.h"

class showVisualizationTestWindow : public QMainWindow
{
    Q_OBJECT
public:
    showVisualizationTestWindow(std::string test);
    ~showVisualizationTestWindow();
    int main();

protected:

    void setupUI();
    Ui::MainWindowShowScene UI;
    SimoxGui::ViewerInterfacePtr viewer;
};

#endif
