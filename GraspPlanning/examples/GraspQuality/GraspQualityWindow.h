
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

#include "../../../Gui/ViewerInterface.h"


#include <vector>

#include "ui_GraspQuality.h"

class GraspQualityWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspQualityWindow(std::string& robotFile, std::string& objectFile);
    ~GraspQualityWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

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
    void objectToTCP();
    void graspQuality();
    void showGWS();
    void showOWS();
    void timerCB();

protected:

    void loadRobot();
    void loadObject();

    void setupUI();

    void updateObject(float x[6]);

    void buildRrtVisu();
    void setEEFComboBox();
    Ui::MainWindowGraspQuality UI;

    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::ModelPtr robot;
    VirtualRobot::ObstaclePtr object;

    VirtualRobot::EndEffectorPtr eef;
    std::vector< VirtualRobot::EndEffectorPtr > eefs;

    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string objectFile;
    std::string eefName;

    GraspPlanning::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    QTimer* timer;
};

#endif
