#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_LoadMesh.h"

class LoadMeshWindow : public QMainWindow
{
    Q_OBJECT
public:
    LoadMeshWindow();
    ~LoadMeshWindow();
    int main();

protected:

    SimoxGui::ViewerInterfacePtr viewer;
    void setupUI();
    Ui::MainWindow UI;
};

