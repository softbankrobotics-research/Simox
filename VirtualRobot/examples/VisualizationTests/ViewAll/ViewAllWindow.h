#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_ViewAll.h"

class ViewAllWindow : public QMainWindow
{
    Q_OBJECT
public:
    ViewAllWindow();
    ~ViewAllWindow();
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

