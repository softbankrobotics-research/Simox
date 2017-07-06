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
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#ifndef _VirtualRobot_PoseConstraint_h_
#define _VirtualRobot_PoseConstraint_h_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/IK/DifferentialIK.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT PoseConstraint : public Constraint, public std::enable_shared_from_this<PoseConstraint>
    {
    public:
        PoseConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet, const SceneObjectPtr& eef, const Eigen::Matrix4f& target,
                       IKSolver::CartesianSelection cartesianSelection = IKSolver::All,
                       float tolerancePosition = 5.0f, float toleranceRotation = 3.0f / 180.0f * M_PI);

        void setVisualization(const SceneObjectSetPtr& visualizationNodeSet);

        Eigen::MatrixXf getJacobianMatrix();
        Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp);
        Eigen::VectorXf getError(float stepSize = 1.0f);
        bool checkTolerances();

        bool getRobotPoseForConstraint(Eigen::Matrix4f& pose);

        const Eigen::Matrix4f& getTarget();

        void updateTarget(const Eigen::Matrix4f& newTarget);

        double optimizationFunction(unsigned int id);
        Eigen::VectorXf optimizationGradient(unsigned int id);

    protected:
        double positionOptimizationFunction();
        Eigen::VectorXf positionOptimizationGradient();

        double orientationOptimizationFunction();
        Eigen::VectorXf orientationOptimizationGradient();

    protected:
        RobotPtr robot;
        RobotNodeSetPtr nodeSet;
        SceneObjectPtr eef;
        Eigen::Matrix4f target;

        DifferentialIKPtr ik;
        IKSolver::CartesianSelection cartesianSelection;

        float tolerancePosition;
        float toleranceRotation;

        SceneObjectSetPtr visualizationNodeSet;

        float lastError;
        float lastLastError;

        float posRotTradeoff;
    };

    typedef std::shared_ptr<PoseConstraint> PoseConstraintPtr;
}

#endif
