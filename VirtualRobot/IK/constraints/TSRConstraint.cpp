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

#include "TSRConstraint.h"

#include <VirtualRobot/MathTools.h>

using namespace VirtualRobot;

#define POSITION_COMPONENT      0
#define ORIENTATION_COMPONENT   1

TSRConstraint::TSRConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet, const RobotNodePtr& eef,
                             const Eigen::Matrix4f& transformation, const Eigen::Matrix4f& eefOffset, const Eigen::Matrix<float, 6, 2>& bounds,
                             float tolerancePosition, float toleranceRotation) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    transformation(transformation),
    eefOffset(eefOffset),
    bounds(bounds),
    toleranceTranslation(1.0f),
    toleranceRotation(0.1f),
    posRotTradeoff(0.1)
{
    ik.reset(new DifferentialIK(nodeSet));

    // Just for setting the TCP (IK will not actually be solved)
    Eigen::Matrix4f goal = Eigen::Matrix4f::Identity();
    ik->setGoal(goal, eef, IKSolver::All, tolerancePosition, toleranceRotation);

    addOptimizationFunction(POSITION_COMPONENT);
    addOptimizationFunction(ORIENTATION_COMPONENT);

    initialized = true;
}

Eigen::MatrixXf TSRConstraint::getJacobianMatrix()
{
    Eigen::VectorXf error = getError();
    Eigen::MatrixXf J = ik->getJacobianMatrix();

    for (int i = 0; i < 6; i++)
    {
        if (error(i) == 0)
        {
            //J.row(i).setZero();
        }
    }

    return J;
}

Eigen::MatrixXf TSRConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    if (tcp->getName() != eef->getName())
    {
        VR_WARNING << "EndEffectorPoseConstraing Jacobian calculation for differing EEF ('" << tcp->getName() << "' instead of '" << eef->getName() << "')" << std::endl;
    }

    return getJacobianMatrix();
}

Eigen::VectorXf TSRConstraint::getError(float stepSize)
{
    float d_w[6];
    Eigen::MatrixXf eef_global = eef->getGlobalPose();
    Eigen::MatrixXf T = transformation.inverse() * eef_global * eefOffset;
    MathTools::eigen4f2rpy(T, d_w);

    Eigen::VectorXf target(6);

    for (int i = 0; i < 6; i++)
    {
        if (d_w[i] < bounds(i, 0))
        {
            target(i) = bounds(i, 0);
        }
        else if (d_w[i] > bounds(i, 1))
        {
            target(i) = bounds(i, 1);
        }
        else
        {
            target(i) = d_w[i];
        }
    }

    Eigen::Matrix4f T_dx;
    MathTools::posrpy2eigen4f(target.block<3, 1>(0, 0), target.block<3, 1>(3, 0), T_dx);

    Eigen::Matrix4f P_target = transformation * T_dx;
    Eigen::Matrix4f P_eef = eef_global * eefOffset;
    Eigen::Matrix4f P_delta = P_target * P_eef.inverse();
    Eigen::AngleAxis<float> P_delta_AA;
    P_delta_AA = P_delta.block<3, 3>(0, 0);

    Eigen::VectorXf dx(6);
    dx.head(3) = Eigen::Vector3f(
                     P_target(0, 3) - P_eef(0, 3),
                     P_target(1, 3) - P_eef(1, 3),
                     P_target(2, 3) - P_eef(2, 3)
                 );
    dx.tail(3) = P_delta_AA.axis() * P_delta_AA.angle();

    return dx * stepSize;
}

bool TSRConstraint::checkTolerances()
{
    Eigen::VectorXf error = getError(1);
    return (error.head(3).norm() < toleranceTranslation) && (error.tail(3).norm() < toleranceRotation);
}

std::string TSRConstraint::getConstraintType()
{
    return "TSR(" + eef->getName() + ")";
}

const Eigen::Matrix4f& TSRConstraint::getTransformation()
{
    return transformation;
}

const Eigen::Matrix<float, 6, 2>& TSRConstraint::getBounds()
{
    return bounds;
}

double TSRConstraint::optimizationFunction(unsigned int id)
{
    switch(id)
    {
        case POSITION_COMPONENT:
            return positionOptimizationFunction();

        case ORIENTATION_COMPONENT:
            return orientationOptimizationFunction();

        default:
            return 0;
    }
}

Eigen::VectorXf TSRConstraint::optimizationGradient(unsigned int id)
{
    switch(id)
    {
        case POSITION_COMPONENT:
            return positionOptimizationGradient();

        case ORIENTATION_COMPONENT:
            return orientationOptimizationGradient();

        default:
            return Eigen::VectorXf();
    }
}

double TSRConstraint::positionOptimizationFunction()
{
    Eigen::VectorXf e = posRotTradeoff * getError().head(3);
    return e.dot(e);
}

Eigen::VectorXf TSRConstraint::positionOptimizationGradient()
{
    Eigen::MatrixXf J = ik->getJacobianMatrix(eef).block(0, 0, 3, nodeSet->getSize());
    Eigen::VectorXf e = posRotTradeoff * getError().head(3);
    return 2 * e.transpose() * J;
}

double TSRConstraint::orientationOptimizationFunction()
{
    Eigen::VectorXf e = getError().tail(3);
    return e.dot(e);
}

Eigen::VectorXf TSRConstraint::orientationOptimizationGradient()
{
    Eigen::MatrixXf J = ik->getJacobianMatrix(eef).block(3, 0, 3, nodeSet->getSize());
    Eigen::VectorXf e = posRotTradeoff * getError().tail(3);
    return 2 * e.transpose() * J;
}

void TSRConstraint::resolveRPYAmbiguities(float* pose, const float* reference)
{
    Eigen::Vector3f ref;
    ref << reference[3], reference[4], reference[5];

    Eigen::Vector3f tmp;

    Eigen::Vector3f best;
    best << pose[3], pose[4], pose[5];

    for (int i = -1; i <= 1; i += 2)
    {
        for (int j = -1; j <= 1; j += 2)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                tmp << reference[3] + float(i) * float(M_PI),
                    reference[4] + float(j) * float(M_PI),
                    reference[5] + float(k) * float(M_PI);

                if ((tmp - ref).norm() < (best - ref).norm())
                {
                    best = tmp;
                }
            }
        }
    }

    pose[3] = best(0);
    pose[4] = best(1);
    pose[5] = best(2);
}

float TSRConstraint::getShortestDistanceForRPYComponent(float from, float to)
{
    float direct = to - from;

    if (direct > M_PI)
    {
        return float(-2 * M_PI + direct);
    }
    else if (direct < -M_PI)
    {
        return float(2 * M_PI - direct);
    }
    else
    {
        return direct;
    }
}
