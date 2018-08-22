
#pragma once

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/IK/GazeIK.h"
#include "VirtualRobot/IK/ConstrainedIK.h"
#include "Gui/AbstractViewer.h"

#include <string.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QTimer>

#include <vector>

#include "ui_GenericIK.h"

class GenericIKWindow : public QMainWindow
{
    Q_OBJECT
public:
    GenericIKWindow(std::string& sRobotFilename);
    ~GenericIKWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void selectIK(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void solve();

    void updateCB();

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);

    Ui::MainWindowGenericIKDemo UI;
    SimoxGui::AbstractViewerPtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::FramePtr tcp;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::GenericIKSolverPtr ikSolver;
    VirtualRobot::GazeIKPtr ikGazeSolver;
    VirtualRobot::ConstrainedIKPtr ikConstrainedSolver;
    VirtualRobot::ObstaclePtr box;

    bool useColModel;
    std::string boxVisuLayer;
    std::string robotVisuLayer;

    QTimer* timer;
};
