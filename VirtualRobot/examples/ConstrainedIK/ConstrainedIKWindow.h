/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#ifndef __Constrained_WINDOW_H_
#define __Constrained_WINDOW_H_

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Obstacle.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/IK/GazeIK.h"
#include "VirtualRobot/IK/ConstrainedIK.h"

#include "VirtualRobot/IK/constraints/PoseConstraint.h"
#include "VirtualRobot/IK/constraints/TSRConstraint.h"
#include "VirtualRobot/IK/constraints/JointLimitAvoidanceConstraint.h"
#include "VirtualRobot/IK/constraints/BalanceConstraint.h"
#include "VirtualRobot/IK/constraints/PositionConstraint.h"
#include "VirtualRobot/IK/constraints/OrientationConstraint.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include "../../../Gui/ViewerFactory.h"

#include <vector>

#include "ui_ConstrainedIK.h"

class ConstrainedIKWindow : public QMainWindow
{
    Q_OBJECT
public:
    ConstrainedIKWindow(std::string& sRobotFilename);
    ~ConstrainedIKWindow();

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

    void solve();

    void updateTSR(double value);
    void randomTSR(bool quiet=false);
    void enableTSR();

    void updatePose(double value);
    void randomPose(bool quiet=false);
    void enablePose();

    void enableBalance();

    void performanceEvaluation();

protected:
    void setupUI();
    void updateKCBox();

    void computePoseError();
    void computeTSRError();

    Ui::MainWindowConstrainedIKDemo UI;
    SimoxGui::AbstractViewerPtr viewer;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::FramePtr tcp;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::PositionConstraintPtr positionConstraint;
    VirtualRobot::OrientationConstraintPtr orientationConstraint;
    VirtualRobot::TSRConstraintPtr tsrConstraint;
    VirtualRobot::BalanceConstraintPtr balanceConstraint;
};

#endif
