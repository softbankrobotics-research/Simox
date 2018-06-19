#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_SimpleCube.h"

class SimpleCubeWindow : public QMainWindow
{
    Q_OBJECT
public:
    SimpleCubeWindow();
    ~SimpleCubeWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr viewer;
    void setupUI();
    Ui::MainWindow UI;
};

