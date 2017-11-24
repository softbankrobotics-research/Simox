
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtOpenGL/QtOpenGL>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SoOffscreenRenderer.h>

#include <vector>

#include "ui_CameraViewer.h"

class showCamWindow : public QMainWindow
{
    Q_OBJECT
public:
    showCamWindow(std::string& sRobotFilename, std::string& cam1Name, std::string& cam2Name);
    ~showCamWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

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


    SoQtExaminerViewer* getExaminerViewer()
    {
        return viewer;
    };

    void updateRobotY(int pos);
    void renderCam();

protected:
    void setupUI();
    void updateJointBox();
    void updateRNSBox();

    void updateCameras();

    void updatRobotInfo();
    Ui::MainWindowCamera UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* extraSep;
    SoSeparator* cam1VoxelSep;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    std::string cam1Name;
    std::string cam2Name;
    VirtualRobot::FramePtr cam1;
    VirtualRobot::FramePtr cam2;
    SoOffscreenRenderer* cam2Renderer;

    std::vector<VirtualRobot::ObstaclePtr> visuObjects;
    std::vector<VirtualRobot::ObstaclePtr> voxelObjects;

    std::vector<unsigned char> cam1RGBBuffer;
    std::vector<float> cam1DepthBuffer;
    std::vector<Eigen::Vector3f> cam1PointCloud;
    unsigned char* cam2Buffer;
    float* cam2DepthBuffer;
    std::vector < VirtualRobot::ModelJointPtr > allRobotNodes;
    std::vector < VirtualRobot::ModelJointPtr > currentRobotNodes;
    std::vector < VirtualRobot::JointSetPtr > robotNodeSets;
    VirtualRobot::JointSetPtr currentRobotNodeSet;
    VirtualRobot::ModelJointPtr currentRobotNode;

    bool useColModel;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;

    struct DepthRenderData
    {
        float* buffer;
        std::size_t w;
        std::size_t h;
    };
    DepthRenderData userdata2;

    static void getDepthImage(void *userdata);
};

#endif
