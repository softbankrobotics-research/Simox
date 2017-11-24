
#ifndef __Jacobi_WINDOW_H_
#define __Jacobi_WINDOW_H_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "Gui/ViewerInterface.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QTimer>

#include <vector>

#include "ui_Jacobi.h"

class JacobiWindow : public QMainWindow
{
    Q_OBJECT
public:
    JacobiWindow(std::string& sRobotFilename);
    ~JacobiWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    SimoxGui::ViewerInterfacePtr getViewer()
    {
        return viewer;
    }

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void jacobiTest();
    void jacobiTest2();
    void jacobiTestBi();
    void updateCB();

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);
    void updatBox2Pos(float x, float y, float z, float a, float b, float g);
    void updatBoxBiPos(float x, float y, float z, float a, float b, float g);



    Ui::MainWindowJacobiDemo UI;
    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::ModelPtr robot;
    std::string robotFilename;
    VirtualRobot::FramePtr tcp;
    VirtualRobot::FramePtr tcp2;
    VirtualRobot::RobotNodePtr elbow;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::ObstaclePtr box;
    VirtualRobot::ObstaclePtr box2;
    VirtualRobot::ObstaclePtr box3;

    bool useColModel;
    std::string boxVisuLayer;
        QTimer* timer;
};

#endif // __Jacobi_WINDOW_H_
