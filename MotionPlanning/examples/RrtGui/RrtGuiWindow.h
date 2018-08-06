
#ifndef __RrtGui_WINDOW_H_
#define __RrtGui_WINDOW_H_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/SceneIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/Model/ManipulationObject.h"

#include "MotionPlanning/MotionPlanning.h"
#include "MotionPlanning/CSpace/CSpacePath.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include "../../../Gui/AbstractViewer.h"
#include "../../../Gui/ViewerFactory.h"

#include <vector>

#include "ui_RrtGui.h"

class RrtGuiWindow : public QMainWindow
{
    Q_OBJECT
public:
    RrtGuiWindow(const std::string& sceneFile, const std::string& sConf, const std::string& gConf, const std::string& rns,
                 const std::string& colModelRob1, const std::string& colModelRob2, const std::string& colModelEnv);
    ~RrtGuiWindow();

    void redraw();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void loadSceneWindow();

    void selectStart(int nr);
    void selectGoal(int nr);
    void selectRNS(int nr);

    void selectColModelRobA(int nr);
    void selectColModelRobB(int nr);
    void selectColModelEnv(VirtualRobot::ModelSetPtr &mns);

    void colModel();
    void solutionSelected();

    void sliderSolution(int pos);

    void buildVisu();

    void plan();

    void timerCB();

protected:

    void loadScene();

    void setupUI();
    void buildRRTVisu();
    void selectStart(const std::string& conf);
    void selectGoal(const std::string& conf);
    void selectRNS(const std::string& rns);

    void buildRrtVisu();
    void selectColModelRobA(const std::string& colModel);
    void selectColModelRobB(const std::string& colModel);
    void selectColModelEnv(const std::string& colModel);

    Ui::MainWindowRRTDemo UI;

    SimoxGui::AbstractViewerPtr viewer;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr robotStart;
    VirtualRobot::RobotPtr robotGoal;

    MotionPlanning::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;
    Eigen::VectorXf goalConfig;

    VirtualRobot::JointSetPtr rns;
    VirtualRobot::LinkSetPtr colModelRobA;
    VirtualRobot::LinkSetPtr colModelRobB;
    std::vector<VirtualRobot::ModelLinkPtr>  colModelEnv;

    std::vector< VirtualRobot::RobotConfigPtr > configs;

    std::string sceneFile;
    VirtualRobot::ScenePtr scene;

    MotionPlanning::CSpacePathPtr solution;
    MotionPlanning::CSpacePathPtr solutionOptimized;
    MotionPlanning::CSpaceTreePtr tree;
    MotionPlanning::CSpaceTreePtr tree2;

    //QTimer* timer;
};

#endif
