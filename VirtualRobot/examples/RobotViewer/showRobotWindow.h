
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include "../../Model/Model.h"
#include "../../VirtualRobotException.h"
#include "../../Model/Nodes/ModelNode.h"
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include "../../XML/ModelIO.h"
#include "../../Visualization/VisualizationFactory.h"
#include "../../Model/Obstacle.h"
#include "../../Model/ModelNodeSet.h"
#include "../../Model/Nodes/Attachments/ModelNodeAttachment.h"

#include "../../../Gui/ViewerInterface.h"
#include "../../../Gui/ViewerFactory.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QToolTip>

#include <vector>

#include "ui_RobotViewer.h"

class showRobotWindow : public QMainWindow
{
    Q_OBJECT
public:
    showRobotWindow(std::string& sRobotFilename);
    ~showRobotWindow();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    virtual void closeEvent(QCloseEvent* event) override;

    void resetRobot();
    void render();
    void loadRobot();
    void attachStructure(bool attach);
    void attachFrames(bool attach);
    void showSensors();
    void closeEEF();
    void openEEF();
    void selectEEF();
    void selectPreshape();
    void selectRobot();
    void attachPhysicsInformation(bool attach);
    void exportXML();
    void displayTriangles();

private:
    void setupUI();
    void updateEEFBox();
    Ui::MainWindowShowRobot UI;

    SimoxGui::ViewerInterfacePtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;

    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;

    bool useColModel;
    bool structureEnabled;
    bool physicsCoMEnabled;
    bool physicsInertiaEnabled;

    std::string robotLayer;

    void testPerformance(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rns);


private slots:
    // updates the joint table and link list based on the selected nodesets.
    void updateModelNodeControls();
    // updates the joint/link set comboboxes based on the current robot
    void updateModelNodeSets();
    // updates all joint values based on all joint table sliders.
    void updateJoints();

private:
    // A slider that adjusts it's tooltip based on current jointvalues
    class JointValueSlider : public QSlider
    {
    public:
        explicit JointValueSlider(VirtualRobot::ModelJointPtr joint, Qt::Orientation orientation, QWidget *parent = 0)
            : QSlider(orientation, parent), joint(joint)
        {
            setMouseTracking(true);
        }
    protected:
        virtual void mouseMoveEvent(QMouseEvent *e) override
        {
            QSlider::mouseMoveEvent(e);
            QString text = QString("min: ") + QString::number(joint->getJointLimitLow()) +
                    QString("\ncurrent: ") + QString::number(joint->getJointValue()) +
                    QString("\nmax: ") + QString::number(joint->getJointLimitHigh());
            QToolTip::showText(mapToGlobal(e->pos()),  text, this);
        }
    private:
        VirtualRobot::ModelJointPtr joint;
    };
};

#endif
