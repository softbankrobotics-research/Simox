
#ifndef __reachabilityScene_WINDOW_H_
#define __reachabilityScene_WINDOW_H_

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <Gui/ViewerInterface.h>

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <VirtualRobot/Workspace/Reachability.h>

#include <vector>

#include "ui_reachabilityScene.h"

class reachabilityWindow : public QMainWindow
{
    Q_OBJECT
public:
    reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP);
    ~reachabilityWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;
    void resetSceneryAll();
    void selectRobot();
    void createReach();
    void saveReach();
    void loadReach();
    void fillHoles();
    void binarize();

    void computeVolume();
    void collisionModel();
    void reachVisu();
    void selectRNS(int nr);
    void selectJoint(int nr);
    void jointValueChanged(int pos);
    void extendReach();

    //void showRobot();
    /*
    void showCoordSystem();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    void closeHand();
    void openHand();
    void selectEEF(int nr);*/

    SimoxGui::ViewerInterfacePtr getViewer() const;

protected:
    void loadRobot();

    void setupUI();
    QString formatString(const char* s, float f);
    void buildVisu();
    void updateRNSBox();
    void updateJointBox();
    void loadReachFile(std::string filename);

    void updateQualityInfo();
    /*
    void updateEEFBox();
    void displayTriangles();*/
    Ui::MainWindowReachability UI;
    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFile;
    std::string reachFile;
    Eigen::Vector3f axisTCP;
    VirtualRobot::JointSetPtr currentRobotNodeSet;
    std::vector < VirtualRobot::ModelJointPtr > allRobotNodes;
    std::vector < VirtualRobot::ModelJointPtr > currentRobotNodes;
    std::vector < VirtualRobot::JointSetPtr > robotNodeSets;

    VirtualRobot::WorkspaceRepresentationPtr reachSpace;
    VirtualRobot::ModelJointPtr currentRobotNode;
    /*

    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    ;*/


    bool useColModel;
    //bool structureEnabled;

    VirtualRobot::VisualizationSetPtr visualization;
    std::string robotVisuLayerName;
    std::string wsVisuLayerName;
};

#endif // __reachabilityScene_WINDOW_H_
