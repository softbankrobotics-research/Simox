
#ifndef __reachabilityScene_WINDOW_H_
#define __reachabilityScene_WINDOW_H_

#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/SceneIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "Gui/ViewerInterface.h"

#include <string.h>
#include <vector>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include "ui_ReachabilityMap.h"

class ReachabilityMapWindow : public QMainWindow
{
    Q_OBJECT
public:
    ReachabilityMapWindow(std::string& sRobotFile, std::string& reachFile, std::string& objFile, std::string& eef);
    ~ReachabilityMapWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);
    void resetSceneryAll();

    void updateVisu();
    void selectEEF();
    void selectGrasp();
    void setObjectRandom();

protected:
    void selectEEF(std::string& eef);
    void selectEEF(int nr);
    void loadRobot();
    void updateEEFBox();
    void buildRobotVisu();
    void buildObjectVisu();
    void buildReachVisu();
    void buildReachGridVisu();
    void buildGraspVisu();

    bool buildReachMapAll();
    bool buildReachMap(VirtualRobot::GraspPtr g);

    void setupUI();
    QString formatString(const char* s, float f);

    void loadReachFile(std::string filename);
    void loadObjectFile(std::string filename);
    void setupEnvironment();

    Ui::MainWindowReachability UI;
    SoQtExaminerViewer* examinerViewer; /*!< Viewer to display the 3D model of the robot and the environment. */
    SimoxGui::ViewerInterfacePtr viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    std::string robotVisuLayer;
    std::string reachVisuLayer;
    std::string reachMapVisuLayer;
    std::string allGraspsVisuLayer;
    std::string graspVisuLayer;
    std::string objectVisuLayer;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::GraspSetPtr grasps;
    VirtualRobot::ManipulationObjectPtr graspObject;
    VirtualRobot::ManipulationObjectPtr environment;
    std::string robotFile;
    std::string reachFile;
    std::string objectFile;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
    std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
    std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;

    VirtualRobot::WorkspaceRepresentationPtr reachSpace;
    VirtualRobot::WorkspaceGridPtr reachGrid;
    VirtualRobot::RobotNodePtr currentRobotNode;

};

#endif // __reachabilityScene_WINDOW_H_
