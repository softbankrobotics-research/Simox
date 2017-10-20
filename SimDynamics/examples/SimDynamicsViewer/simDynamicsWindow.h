
#ifndef __SimDynamics_WINDOW_H_
#define __SimDynamics_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include <VirtualRobot/Model/Nodes/ModelJointRevolute.h>

#include "SimDynamics/DynamicsEngine/BulletEngine/BulletCoinQtViewer.h"

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
#include <atomic>

#include "ui_simDynamicsViewer.h"

class SimDynamicsWindow : public QMainWindow
{
    Q_OBJECT
public:
    SimDynamicsWindow(std::string& sRobotFilename);
    ~SimDynamicsWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void buildVisualization();
    void actuation();

    void loadButton();

    void selectRobotNode(int n);
    void jointValueChanged(int n);
    void fixedTimeStepChanged(int n);
    void updateTimerChanged(int n);
    void updateAntiAliasing(int n);
    void comVisu();
    void updateJointInfo();
    void updateRobotInfo();

    void startStopEngine();
    void stepEngine();

    void checkBoxFixedTimeStep();

    void addObject();

    void reloadRobot();

    void resetPose();
    void setPose();

protected:
    bool loadRobot(std::string robotFilename);
    void setupUI();
    void updateJoints();

    void stopCB();

    void updateContactVisu();
    void updateComVisu();

    SimDynamics::DynamicsWorldPtr dynamicsWorld;
    SimDynamics::DynamicsModelPtr dynamicsRobot;
    SimDynamics::DynamicsModelPtr dynamicsObject;
    SimDynamics::DynamicsObjectPtr dynamicsObject2;
    std::vector<SimDynamics::DynamicsModelPtr> dynamicsObjects;

    Ui::MainWindowBulletViewer UI;

    SoSeparator* sceneSep;
    SoSeparator* comSep;
    SoSeparator* contactsSep;
    SoSeparator* forceSep;

    SimDynamics::BulletCoinQtViewerPtr viewer;

    VirtualRobot::RobotPtr robot;

    // beside the viewer cb we need also a callback to update joint info
    static void timerCB(void* data, SoSensor* sensor);
    static void stepCB(void* data, btScalar timeStep);

    SoTimerSensor* timerSensor;

    std::vector<VirtualRobot::ModelJointPtr> robotNodes;

    std::map< VirtualRobot::ModelLinkPtr, SoSeparator* > comVisuMap;

    bool useColModel;
    std::string robotFilename;

    std::atomic_uint_fast64_t simuStepCount{0};
};

#endif // __SimDynamics_WINDOW_H_
