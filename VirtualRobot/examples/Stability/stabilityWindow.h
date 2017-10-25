
#ifndef __stabilityScene_WINDOW_H_
#define __stabilityScene_WINDOW_H_

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Model/Obstacle.h>

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

#include "ui_stabilityScene.h"

class stabilityWindow : public QMainWindow
{
    Q_OBJECT
public:
    stabilityWindow(const std::string& robotFile, const std::string linkset, const std::string &jointset);
    ~stabilityWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();
    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);
    void resetSceneryAll();
    void selectRobot();
    void collisionModel();
    void selectJoint(int nr);
    void jointValueChanged(int pos);
    void selectJointSet(int nr);
    void selectLinkSet(int nr);
    void showCoM();
    void showSupportPolygon();
    void performCoMIK();
    void comTargetMovedX(int value);
    void comTargetMovedY(int value);


protected:
    void loadRobot();
    void buildVisu();

    void setupUI();
    QString formatString(const char* s, float f);

    void updateJointBox();
    void updateRNSBox();
    void updateCoM();
    void updateSupportVisu();

    void selectJointSet(const std::string &jointset);
    void selectLinkSet(const std::string &linkset);

    Ui::MainWindowStability UI;
    SoQtExaminerViewer* m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    Eigen::Vector2f m_CoMTarget;

    SoSeparator* sceneSep;
    SoSeparator* robotVisuSep;
    SoSeparator* comVisu;
    SoSeparator* comProjectionVisu;
    SoSeparator* comTargetVisu;
    SoSeparator* supportVisu;

    VirtualRobot::RobotPtr robot;
    std::string robotFile;

    VirtualRobot::LinkSetPtr currentLinkSet;
    VirtualRobot::JointSetPtr currentJointSet;
    std::vector < VirtualRobot::ModelJointPtr > allJoints;
    std::vector < VirtualRobot::ModelJointPtr > currentJoints;
    std::vector < VirtualRobot::ModelLinkPtr > allLinks;
    std::vector < VirtualRobot::ModelLinkPtr > currentLinks;
    std::vector < VirtualRobot::LinkSetPtr > linkSets;
    std::vector < VirtualRobot::JointSetPtr > jointSets;

    VirtualRobot::ModelJointPtr currentJoint;


    bool useColModel;


    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};

#endif // __stabilityScene_WINDOW_H_
