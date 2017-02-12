#ifndef __Platform_WINDOW_H__
#define __Platform_WINDOW_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>

#include "MotionPlanning/Saba.h"
#include "MotionPlanning/CSpace/CSpacePath.h"

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

#include "ui_PlatformDemo.h"

class PlatformWindow : public QMainWindow
{
    Q_OBJECT
public:
    PlatformWindow(const std::string& sceneFile,
                   const std::string& rns,
                   const std::string& colModelRob,
                   const std::string& colModelEnv);
    ~PlatformWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void loadSceneWindow();


    void colModel();
    void solutionSelected();

    void sliderSolution(int pos);

    void buildVisu();
    void optimizeSolutionPressed();


    void plan();

protected:

    struct planSet
    {
        std::string rns;
        std::string colModelRob;
        std::string colModelEnv;
    };

    planSet planSetA;

    enum postProcessingMethod
    {
        eShortcuts,
        eElasticBands
    };



    void loadScene();
    void optimizeSolution(postProcessingMethod postProcessing, int nrSteps);

    void setupUI();
    QString formatString(const char* s, float f);
    void buildRRTVisu();

    void selectRNS(const std::string& rns);
    void setStart(Eigen::VectorXf &goalConf);
    void setGoal(Eigen::VectorXf &goalConf);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();
    void selectColModelRob(const std::string& colModel);
    void selectColModelEnv(const std::string& colModel);

    void updateDistVisu(const Eigen::Vector3f &a, const Eigen::Vector3f &b);

    void showOptizerForces(Saba::ElasticBandProcessorPtr postProcessing, Saba::CSpacePathPtr s);

    Ui::MainWindowPlatformdemo UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* allSep;
    SoSeparator* sceneFileSep;
    SoSeparator* rrtSep;
    SoSeparator* distSep;
    SoSeparator* forcesSep;

    VirtualRobot::RobotPtr robot;

    Saba::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;
    Eigen::VectorXf goalConfig;

    VirtualRobot::RobotNodeSetPtr rns;
    //VirtualRobot::SceneObjectSetPtr colModelRob;
    VirtualRobot::RobotNodeSetPtr colModelRob;
    VirtualRobot::SceneObjectSetPtr colModelEnv;

    std::vector< VirtualRobot::RobotConfigPtr > configs;
    std::vector< VirtualRobot::ObstaclePtr > obstacles;


    std::string sceneFile;
    VirtualRobot::ScenePtr scene;

    VirtualRobot::CDManagerPtr cdmPlayback;


    Saba::CSpacePathPtr solution;
    Saba::CSpacePathPtr solutionOptimized;
    Saba::CSpaceTreePtr tree;
    Saba::CSpaceTreePtr tree2;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;

    Saba::BiRrtPtr rrt;
};

#endif // __Platform_WINDOW_H__
