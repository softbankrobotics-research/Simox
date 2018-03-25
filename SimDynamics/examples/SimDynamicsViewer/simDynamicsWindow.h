
#ifndef __SimDynamics_WINDOW_H_
#define __SimDynamics_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include <VirtualRobot/Model/Nodes/ModelJointRevolute.h>

#include "SimDynamics/DynamicsEngine/BulletEngine/BulletCoinQtViewer.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QTimer>

#include <vector>
#include <atomic>

#include "ui_simDynamicsViewer.h"

class SimDynamicsWindow : public QMainWindow
{
    Q_OBJECT
public:
    SimDynamicsWindow(std::string& sRobotFilename);
    ~SimDynamicsWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

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

    // beside the viewer cb we need also a callback to update joint info
    void timerCB();

protected:
    static void stepCB(void* data, btScalar timeStep);
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
#ifdef Simox_USE_COIN_VISUALIZATION
    SimDynamics::BulletCoinQtViewerPtr viewer;
#else
    // todo qt3d viewer
#endif
    VirtualRobot::ModelPtr robot;

    std::vector<VirtualRobot::ModelJointPtr> robotNodes;

    //std::map< VirtualRobot::ModelLinkPtr, VirtualRobot::VisualizationNodePtr > comVisuMap;

    bool useColModel;
    std::string robotFilename;

    QTimer* timer;

    std::atomic_uint_fast64_t simuStepCount{0};
};

#endif // __SimDynamics_WINDOW_H_
