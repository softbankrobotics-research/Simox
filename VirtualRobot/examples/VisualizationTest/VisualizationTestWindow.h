#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_VisualizationTest.h"

class VisualizationTestWindow : public QMainWindow
{
    Q_OBJECT
public:
    VisualizationTestWindow();
    ~VisualizationTestWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr viewer;
    void setupUI();
    Ui::MainWindow UI;
};

