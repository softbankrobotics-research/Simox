
#ifndef __MTPlanning_WINDOW_H_
#define __MTPlanning_WINDOW_H_

#include "MotionPlanning/MotionPlanning.h"

#include "MTPlanningScenery.h"
#include <VirtualRobot/Model/Model.h>

#include <qobject.h>
#include <qmainwindow.h>
#include <qpushbutton.h>
#include <qtextedit.h>
#include <qcombobox.h>
#include <qprogressbar.h>
#include <qlabel.h>
#include <qslider.h>
#include <qcheckbox.h>

#include <QTimer>

#include <string.h>
#include <time.h>

#include "../../../Gui/AbstractViewer.h"
#include "../../../Gui/ViewerFactory.h"

#include "ui_MTPlanning.h"

#define NUMBER_OF_PLANNING 30

class MTPlanningWindow : public QMainWindow
{
    Q_OBJECT

public:
    MTPlanningWindow(const std::string &robotFile);
    ~MTPlanningWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    //void loadRobot();
    void buildScene();
    void addThread();
    void startThreads();
    void stopThreads();
    void startOptimize();
    void stopOptimize();
    void reset();
    void selectColCheckerComboBoxChanged(int value);

    void timerCBPlanning();
    void timerCBOptimize();

protected:
    void setupLayoutMTPlanning(const std::string &robotFile); /*!< Create the contents of the window. */

    QTimer* timer1;
    QTimer* timer2;

    Ui::MainWindowMTPlanning UI;
    SimoxGui::AbstractViewerPtr viewer;

    MTPlanningScenery* scene;

    clock_t startTime;
    clock_t endTime;
    clock_t optiStartTime;
    clock_t optiEndTime;
};

#endif
