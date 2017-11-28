
#ifndef __IKRRT_WINDOW_H_
#define __IKRRT_WINDOW_H_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/SceneIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/Model/ManipulationObject.h"

#include "MotionPlanning/MotionPlanning.h"
#include "MotionPlanning/CSpace/CSpacePath.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include "../../../Gui/ViewerInterface.h"
#include "../../../Gui/ViewerFactory.h"

#include <vector>

#include "ui_IKRRT.h"

class IKRRTWindow : public QMainWindow
{
    Q_OBJECT
public:
    IKRRTWindow(std::string& sceneFile, std::string& reachFile, std::string& rns, std::string& eef, std::string& colModel, std::string& colModelRob);
    ~IKRRTWindow();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void searchIK();

    void colModel();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();
    void sliderReleased_ObjectA();
    void sliderReleased_ObjectB();
    void sliderReleased_ObjectG();
    void sliderSolution(int pos);

    void buildVisu();

    void showCoordSystem();
    void reachVisu();

    void planIKRRT();

    void playAndSave();

    void timerCB();

protected:

    void loadScene();
    void loadReach();

    void setupUI();

    void buildGraspSetVisu();

    void buildRRTVisu();

    void updateObject(float x[6]);

    void buildRrtVisu();
    void saveScreenshot();
    Ui::MainWindowIKRRT UI;
    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::RobotPtr robot;
    std::vector< VirtualRobot::ObstaclePtr > obstacles;
    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::ReachabilityPtr reachSpace;

    VirtualRobot::EndEffectorPtr eef;
    MotionPlanning::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;

    VirtualRobot::GraspSetPtr graspSet;
    VirtualRobot::JointSetPtr rns;

    std::string sceneFile;
    std::string reachFile;
    std::string eefName;
    std::string rnsName;
    std::string colModelName;
    std::string colModelNameRob;

    MotionPlanning::CSpacePathPtr solution;
    MotionPlanning::CSpacePathPtr solutionOptimized;
    MotionPlanning::CSpaceTreePtr tree;
    MotionPlanning::CSpaceTreePtr tree2;

    bool playbackMode;
    int playCounter;

    QTimer* timer;

    //VirtualRobot::VisualizationPtr visualizationRobot;
    //VirtualRobot::VisualizationPtr visualizationObject;
};

#endif
