
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/Model/ManipulationObject.h>

#include "GraspPlanning/GraspPlanning.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "GraspPlanning/GraspPlanner/GenericGraspPlanner.h"
#include "GraspPlanning/ApproachMovementSurfaceNormal.h"
#include "GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h"

#include <string.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Gui/AbstractViewer.h>

#include <vector>

#include "ui_GraspPlanner.h"

class GraspPlannerWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspPlannerWindow(std::string& robotFile, std::string& eefName, std::string& preshape, std::string& objectFile);
    ~GraspPlannerWindow() override;

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void colModel();
    void frictionConeVisu();
    void showGrasps();

    void buildVisu();

    void plan();
    void save();

    void planObjectBatch();
protected:
    bool evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspPlanning::GraspEvaluationPoseUncertainty::PoseEvalResults &results);

    void loadRobot();
    void loadObject();

    void setupUI();

    Ui::GraspPlanner UI;

    SimoxGui::AbstractViewerPtr viewer;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr eefCloned;
    VirtualRobot::ObstaclePtr object;
    VirtualRobot::EndEffectorPtr eef;

    VirtualRobot::GraspSetPtr grasps;


    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string objectFile;
    std::string eefName;
    std::string preshape;

    GraspPlanning::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
    GraspPlanning::ApproachMovementSurfaceNormalPtr approach;
    GraspPlanning::GenericGraspPlannerPtr planner;

    VirtualRobot::VisualizationPtr visualizationRobot;
    VirtualRobot::VisualizationPtr visualizationObject;
};

