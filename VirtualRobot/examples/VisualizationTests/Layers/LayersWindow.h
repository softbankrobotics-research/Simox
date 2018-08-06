#include <QMainWindow>

#include "../../../Gui/ViewerFactory.h"

#include "ui_Layers.h"

class LayersWindow : public QMainWindow
{
    Q_OBJECT
public:
    LayersWindow();
    ~LayersWindow();
    int main();

protected:

    SimoxGui::AbstractViewerPtr viewer;
    void setupUI();
    Ui::MainWindow UI;
    bool toggle;

public slots:
    void update();
};

