
#ifndef __GraspEditor_WINDOW_H_
#define __GraspEditor_WINDOW_H_

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/Model/ManipulationObject.h>
#include <string.h>
#include <QtGlobal>
#if QT_VERSION >= 0x050000
#include <QtWidgets/QtWidgets>
#else
#include <QtGui>
#endif
#include <QtCore/QtCore>
#include <QTimer>

#include "../../../Gui/ViewerFactory.h"

#include <vector>

// #include "ui_GraspEditor.h"

namespace Ui
{
    class MainWindowGraspEditor;
}


namespace VirtualRobot
{

    class SIMOX_GUI_IMPORT_EXPORT GraspEditorWindow : public QMainWindow
    {
        Q_OBJECT
    public:
        GraspEditorWindow(std::string& objFile, std::string& robotFile, bool embeddedGraspEditor = false);
        virtual ~GraspEditorWindow();

    public slots:
        /*! Closes the window and exits SoQt runloop. */
        void quit();

        /*!< Overriding the close event, so we know when the window was closed by the user. */
        void closeEvent(QCloseEvent* event);

        void loadObject();
        void loadRobot();

        void selectRobot();
        void selectObject(std::string file = "");
        void saveObject();
        void selectEEF(int n);
        void selectGrasp(int n);

        void resetSceneryAll();

        void closeEEF();
        void openEEF();

        void addGrasp();
        void renameGrasp();

        void sliderReleased_ObjectX();
        void sliderReleased_ObjectY();
        void sliderReleased_ObjectZ();
        void sliderReleased_ObjectA();
        void sliderReleased_ObjectB();
        void sliderReleased_ObjectG();

        void buildVisu();

        void showCoordSystem();

        void timerCB();

    protected:

        void setupUI();

        void updateEEFBox();
        void updateGraspBox();

        void buildGraspSetVisu();

        void updateEEF(float x[6]);

        void setCurrentGrasp(Eigen::Matrix4f& p);

        Ui::MainWindowGraspEditor* UI;

        // Indicates whether this program is started embedded
        bool embeddedGraspEditor;

        SimoxGui::ViewerInterfacePtr viewer;

        VirtualRobot::RobotPtr robot;
        VirtualRobot::RobotPtr robotEEF;
        VirtualRobot::ManipulationObjectPtr object;
        std::vector<VirtualRobot::EndEffectorPtr> eefs;
        VirtualRobot::EndEffectorPtr currentEEF; // the eef of robot
        VirtualRobot::EndEffectorPtr robotEEF_EEF; // the eef of robotEEF

        VirtualRobot::GraspSetPtr currentGraspSet;
        VirtualRobot::GraspPtr currentGrasp;

        std::string robotFile;
        std::string objectFile;

        QTimer* timer;
    };

}
#endif // __GraspEditor_WINDOW_H_
