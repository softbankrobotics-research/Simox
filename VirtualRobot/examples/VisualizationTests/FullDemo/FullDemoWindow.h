#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_FullDemo.h"

class FullDemoWindow : public QMainWindow
{
    Q_OBJECT
public:
    FullDemoWindow();
    ~FullDemoWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr factoryDemoViewer;
    SimoxGui::ViewerInterfacePtr viewer2;

    void setupUI();
    Ui::MainWindow UI;
};

