#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_Screenshot.h"

class ScreenshotWindow : public QMainWindow
{
    Q_OBJECT
public:
    ScreenshotWindow();
    ~ScreenshotWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr viewer;
    void setupUI();
    Ui::MainWindow UI;
    VirtualRobot::VisualizationPtr visu;

private slots:
    void on_pushButton_2_clicked();
};

