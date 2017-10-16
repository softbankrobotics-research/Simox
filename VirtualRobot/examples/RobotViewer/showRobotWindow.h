
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include "../../Model/Model.h"
#include "../../VirtualRobotException.h"
#include "../../Model/Nodes/ModelNode.h"
#include "../../XML/ModelIO.h"
#include "../../Visualization/VisualizationFactory.h"
#include "../../Model/Obstacle.h"
#include "../../Model/ModelNodeSet.h"
#include "../../Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "../../Visualization/CoinVisualization/CoinVisualization.h"
#include "../../Model/Nodes/Attachments/ModelNodeAttachment.h"

#include "../../../Gui/ViewerInterface.h"
#include "../../../Gui/ViewerFactory.h"
#include "../../../Gui/ViewerSelection.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <vector>

#include "ui_RobotViewer.h"

class showRobotWindow : public QMainWindow
{
    Q_OBJECT
public:
    showRobotWindow(std::string& sRobotFilename);
    ~showRobotWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void rebuildVisualization();
    void loadRobot();
    void selectJoint(int nr);
    void selectRNS(int nr);
    void jointValueChanged(int pos);
    void showCoordSystem();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    void showSensors();
    void closeHand();
    void openHand();
    void selectEEF(int nr);
    void selectRobot();
    void displayPhysics();
    void exportVRML();
    void exportXML();

protected:
    void setupUI();
    void updateJointBox();
    void updateRNSBox();
    void updateEEFBox();
    void displayTriangles();
    void updatRobotInfo();
    Ui::MainWindowShowRobot UI;

    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;

    std::vector < VirtualRobot::ModelNodePtr > allNodes;
    std::vector < VirtualRobot::ModelNodePtr > currentNodes;

    std::vector < VirtualRobot::ModelNodeSetPtr > robotNodeSets;
    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    VirtualRobot::ModelNodeSetPtr currentRobotNodeSet;
    VirtualRobot::ModelNodePtr currentRobotNode;

    bool useColModel;
    bool structureEnabled;
    bool physicsCoMEnabled;
    bool physicsInertiaEnabled;

    void testPerformance(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rns);
};

#endif
