#include "VisualizationTestWindow.h"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication testWindowApplication(argc, argv);

    VisualizationTestWindow testWindow;
    testWindow.show();

    return testWindowApplication.exec();
}
