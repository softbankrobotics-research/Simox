#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_VisualizationLifetime.h"

class VisualizationLifetimeWindow : public QMainWindow
{
    Q_OBJECT
public:
    VisualizationLifetimeWindow();
    ~VisualizationLifetimeWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr viewer;
    void setupUI();
    Ui::MainWindow UI;
    bool toggle;
    VirtualRobot::VisualizationPtr visu;

public slots:
    void update();
};

