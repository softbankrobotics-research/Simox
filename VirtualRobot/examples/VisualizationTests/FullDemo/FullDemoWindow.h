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
    SimoxGui::ViewerInterfacePtr screenshotViewer;

    void setupUI();
    Ui::MainWindow UI;
private slots:
    void on_pushButton_clicked();
};

