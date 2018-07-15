
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/Model/JointSet.h>

#include <Gui/ViewerFactory.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <vector>

#include "ui_CameraViewer.h"

class showCamWindow : public QMainWindow
{
    Q_OBJECT
public:
    showCamWindow(std::string& sRobotFilename, std::string& cam1Name, std::string& cam2Name);
    ~showCamWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void rebuildVisualization();
    void showRobot();
    void loadRobot();
    void selectJoint(int nr);
    void selectRNS(int nr);
    void jointValueChanged(int pos);


    void selectRobot();

    void updateRobotY(int pos);
    void renderCam();

protected:
    void setupUI();
    void updateJointBox();
    void updateRNSBox();

    void updateCameras();

    void updatRobotInfo();
    Ui::MainWindowCamera UI;
    SimoxGui::ViewerInterfacePtr viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    VirtualRobot::VisualizationSetPtr obstacleVisu;
    VirtualRobot::VisualizationPtr robotVisu;
    VirtualRobot::VisualizationPtr cam1pclVisu;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    std::string cam1Name;
    std::string cam2Name;
    VirtualRobot::FramePtr cam1;
    VirtualRobot::FramePtr cam2;

    std::vector<VirtualRobot::ObstaclePtr> visuObjects;
    std::vector<VirtualRobot::ObstaclePtr> voxelObjects;

    std::vector<unsigned char> cam1RGBBuffer;
    std::vector<float> cam1DepthBuffer;
    std::vector<Eigen::Vector3f> cam1PointCloud;
    std::vector<unsigned char> cam2RGBBuffer;
    std::vector<float> cam2DepthBuffer;
    std::vector<Eigen::Vector3f> cam2PointCloud;

    std::vector < VirtualRobot::ModelJointPtr > allRobotNodes;
    std::vector < VirtualRobot::ModelJointPtr > currentRobotNodes;
    std::vector < VirtualRobot::JointSetPtr > robotNodeSets;
    VirtualRobot::JointSetPtr currentRobotNodeSet;
    VirtualRobot::ModelJointPtr currentRobotNode;

    bool useColModel;
};

#endif
