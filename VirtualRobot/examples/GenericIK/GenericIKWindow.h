
#ifndef __Generic_WINDOW_H_
#define __Generic_WINDOW_H_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/IK/GazeIK.h"
#include "VirtualRobot/IK/ConstrainedIK.h"
#include "Gui/ViewerInterface.h"

#include <string.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "ui_GenericIK.h"

class GenericIKWindow : public QMainWindow
{
    Q_OBJECT
public:
    GenericIKWindow(std::string& sRobotFilename);
    ~GenericIKWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void selectIK(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void solve();


protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);

    static void updateCB(void* data, SoSensor* sensor);


    Ui::MainWindowGenericIKDemo UI;
    SoQtExaminerViewer* exViewer; /*!< Viewer to display the 3D model of the robot and the environment. */
    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::FramePtr tcp;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::GenericIKSolverPtr ikSolver;
    VirtualRobot::GazeIKPtr ikGazeSolver;
    VirtualRobot::ConstrainedIKPtr ikConstrainedSolver;
    VirtualRobot::ObstaclePtr box;

    bool useColModel;
    std::string boxVisuLayer;
    std::string robotVisuLayer;
};

#endif // __Generic_WINDOW_H_
