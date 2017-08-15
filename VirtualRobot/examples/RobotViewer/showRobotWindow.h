
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include "../../Model/Model.h"
#include "../../VirtualRobotException.h"
#include "../../Model/Nodes/ModelNode.h"
#include "../../XML/RobotIO.h"
#include "../../Visualization/VisualizationFactory.h"
#include "../../Model/Obstacle.h"
#include "../../Model/ModelNodeSet.h"
#include "../../Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "../../Visualization/CoinVisualization/CoinVisualization.h"

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
    void showRobot();
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

    SoQtExaminerViewer* getExaminerViewer()
    {
        return viewer;
    };

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateJointBox();
    void updateRNSBox();
    void updateEEFBox();
    void displayTriangles();
    void updatRobotInfo();
    Ui::MainWindowShowRobot UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* extraSep;

    VirtualRobot::RobotPtr robot;
    std::string m_sRobotFilename;
    std::vector < VirtualRobot::ModelJointPtr > allJoints;
    std::vector < VirtualRobot::ModelJointPtr > currentJoints;
    std::vector < VirtualRobot::ModelLinkPtr > currentLinks;
    std::vector < VirtualRobot::ModelNodeSetPtr > robotNodeSets;
    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    VirtualRobot::RobotNodePtr currentRobotNode;


    bool useColModel;
    bool structureEnabled;
    bool physicsCoMEnabled;
    bool physicsInertiaEnabled;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;

    void testPerformance(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rns);
};

#endif // __ShowRobot_WINDOW_H_
