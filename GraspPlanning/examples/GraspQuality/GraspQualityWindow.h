
#ifndef __GraspQuality_WINDOW_H_
#define __GraspQuality_WINDOW_H_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/SceneIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/Model/ManipulationObject.h"

#include "GraspPlanning/GraspPlanning.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QTimer>

#include "../../../Gui/AbstractViewer.h"


#include <vector>

#include "ui_GraspQuality.h"

class GraspQualityWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspQualityWindow(std::string& robotFile, std::string& objectFile);
    ~GraspQualityWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void colModel();
    void frictionConeVisu();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();
    void sliderReleased_ObjectA();
    void sliderReleased_ObjectB();
    void sliderReleased_ObjectG();

    void buildVisu();

    void selectEEF(int nr);
    void selectGrasp(int nr);
    void objectToTCP();
    void objectToGrasp();

    void graspQuality();
    void showGWS();
    void showOWS();
    void timerCB();

    void selectObject();

    void evalRobustness();

protected:

    void loadRobot();
    void loadObject();

    void setupUI();

    void updateObject(float x[6]);

    void buildRrtVisu();
    void setEEFComboBox();
    void setGraspComboBox();

    Ui::MainWindowGraspQuality UI;

    SimoxGui::AbstractViewerPtr viewer;

    VirtualRobot::ModelPtr robot;
    VirtualRobot::ManipulationObjectPtr object;


    VirtualRobot::EndEffectorPtr eef;
    std::vector< VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::GraspPtr grasp;
    VirtualRobot::GraspSetPtr grasps;

    std::vector<Eigen::Matrix4f> evalPoses;

    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string objectFile;
    std::string eefName;

    GraspPlanning::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    QTimer* timer;
};

#endif
