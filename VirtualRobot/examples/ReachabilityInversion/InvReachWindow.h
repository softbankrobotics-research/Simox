
#ifndef __InvReachScene_WINDOW_H_
#define __InvReachScene_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/IK/PoseQualityExtendedManipulability.h>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include <VirtualRobot/Workspace/ReachabilityInversion/OrientedWorkspaceGrid.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/InverseReachability.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/RobotPlacementIK.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/RobotPlacementTrajectoryIK.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/Visualization/InverseReachabilityCoinVisualization.h>

#include <vector>

#include "ui_InvReachDemo.h"

#define MANIPULABILITY

//#define TEST_2D
//#define ENDLESS

#ifdef TEST_2D
#undef MANIPULABILITY
#endif

class InvReachWindow : public QMainWindow
{
	Q_OBJECT
public:
	InvReachWindow(std::string &sRobotFile,std::string &reachFile,std::string &objFile, std::string &invReachFile, std::string &rnsNameCollisionDetection,
        std::string &rnFootL, std::string &rnFootR, std::vector<std::string> environments, Qt::WFlags flags = 0);
	~InvReachWindow();

	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);
	void resetSceneryAll();

	void updateVisu();
	void selectTrajectory();
	void selectEEF();
	void selectScene();
	void selectGrasp();
	void setObjectRandom();
	void createFullReachGrid();
	void test1();
	void ikRobotPose();
	void ikAll();
	void testTrajectory();
	void trajectorySolutionUpdate();

protected:
	float getBaseHeight();
	Eigen::Matrix4f getTargetPose();
	void selectEEF(std::string &eef);
	void selectEEF(int nr);
	void selectScene(int nr);
	void loadRobot();
	void updateEEFBox();
	void buildObject();
    void doIK(bool output, bool considerCollisions = true, int minQuality = -1);

    void createRobotPlacementIK(bool lazyGridUpdate);
	void createRobotPlacementTrajectoryIK(bool lazyGridUpdate);

	bool buildReachMapAll();
	bool buildReachMap();

	void setupUI();
	QString formatString(const char *s, float f);

	void loadReachFile(std::string filename);
	void loadObjectFile(std::string filename);
	void setupEnvironment(int nr);
	void calculateManipulability(bool output);
	Ui::MainWindowReachability UI;
	SoQtExaminerViewer *viewer; /*!< Viewer to display the 3D model of the robot and the environment. */
		
	SoSeparator *sceneSep;
	SoSeparator *robotVisuSep;
	SoSeparator *reachabilityVisuSep;
	SoSeparator *reachabilityMapVisuSep;
	SoSeparator *allGraspsVisuSep;
	SoSeparator *targetVisuSep;
	SoSeparator *objectVisuSep;
	SoSeparator *trajectoryVisuSep;
	SoSeparator *footLRVisuSep;
	SoSeparator *reachableFootVisuSep;

	Eigen::Matrix4f trafoBaseToFootL;	
	Eigen::Matrix4f trafoBaseToFootR;

	VirtualRobot::RobotPtr robot;
	VirtualRobot::EndEffectorPtr eef;
	VirtualRobot::RobotNodePtr footL;
	VirtualRobot::RobotNodePtr footR;
	VirtualRobot::GraspSetPtr grasps;
	VirtualRobot::ObstaclePtr graspObject;
	VirtualRobot::ManipulationObjectPtr environment;
    //VirtualRobot::ScenePtr pipelineScene; // only needed for pipeline mode
	std::string robotFile;
	std::string reachFile;
	std::string invReachFile;
	std::string rnsNameCollisionDetection;
	VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
	VirtualRobot::SceneObjectSetPtr currentRobotNodeColSet;
	std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
	std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
	std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;	

    std::vector< std::string > envFiles;

#ifdef MANIPULABILITY
	VirtualRobot::ManipulabilityPtr reachSpace;
#else
	VirtualRobot::ReachabilityPtr reachSpace;
#endif
	VirtualRobot::OrientedWorkspaceGridPtr reachGrid;
	VirtualRobot::RobotNodePtr currentRobotNode;
	VirtualRobot::RobotPlacementTrajectoryIK::TrajectoryIK trajectorySolution;
	float currentAlpha;
	Eigen::Matrix4f originalObjPose;
    bool performanceTest;

    VirtualRobot::RobotPtr eefClone;

	VirtualRobot::InverseReachabilityPtr invReach;

	VirtualRobot::RobotPlacementIKPtr robotPlacementIK;
	VirtualRobot::RobotPlacementTrajectoryIKPtr robotPlacementTrajectoryIK;
	int currentScene;

	std::map< std::string, std::vector< Eigen::Matrix4f > > trajectories;

	VirtualRobot::PoseQualityExtendedManipulabilityPtr manip;

};

#endif // __InvReachScene_WINDOW_H_
