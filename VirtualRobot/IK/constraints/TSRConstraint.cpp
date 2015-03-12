#include "TSRConstraint.h"

#include <VirtualRobot/MathTools.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCube.h>

using namespace VirtualRobot;

TSRConstraint::TSRConstraint(const RobotPtr &robot, const RobotNodeSetPtr &nodeSet, const RobotNodePtr &eef,
                             const Eigen::Matrix4f &transformation, const Eigen::Matrix4f &eefOffset, const Eigen::Matrix<float, 6, 2> &bounds,
                             float tolerancePosition, float toleranceRotation) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    transformation(transformation),
    eefOffset(eefOffset),
    bounds(bounds),
    toleranceTranslation(1.0f),
    toleranceRotation(0.1f)
{
    ik.reset(new DifferentialIK(nodeSet));

    // Just for setting the TCP (IK will not actually be solved)
    Eigen::Matrix4f goal = Eigen::Matrix4f::Identity();
    ik->setGoal(goal, eef, IKSolver::All, tolerancePosition, toleranceRotation);

    initialized = true;
}

Eigen::MatrixXf TSRConstraint::getJacobianMatrix()
{
    Eigen::VectorXf error = getError();
    Eigen::MatrixXf J = ik->getJacobianMatrix();

    for(int i = 0; i < 6; i++)
    {
        if(error(i) == 0)
        {
            J.row(i).setZero();
        }
    }

    return J;
}

Eigen::MatrixXf TSRConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    if(tcp->getName() != eef->getName())
    {
        VR_WARNING << "EndEffectorPoseConstraing Jacobian calculation for differing EEF ('" << tcp->getName() << "' instead of '" << eef->getName() << "')" << std::endl;
    }

    return getJacobianMatrix();
}

Eigen::VectorXf TSRConstraint::getError(float stepSize)
{
    float eef_pose[6], target[6];
    MathTools::eigen4f2rpy(eef->getGlobalPose() * eefOffset, eef_pose);
    MathTools::eigen4f2rpy(transformation, target);

    Eigen::VectorXf error(6);
    for(int i = 0; i < 6; i++)
    {
        if(eef_pose[i] < target[i] + bounds(i,0))
        {
            error(i) = target[i] + bounds(i,0) - eef_pose[i];
        }
        else if(eef_pose[i] > target[i] + bounds(i,1))
        {
            error(i) = target[i] + bounds(i,1) - eef_pose[i];
        }
        else
        {
            error(i) = 0;
        }
    }

    return error * stepSize;
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

const Eigen::Matrix4f &TSRConstraint::getTransformation()
{
    return transformation;
}

const Eigen::Matrix<float, 6, 2> &TSRConstraint::getBounds()
{
    return bounds;
}


