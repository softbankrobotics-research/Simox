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
            //J.row(i).setZero();
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
    float d_w[6];
    Eigen::MatrixXf eef_global = eef->getGlobalPose();
    Eigen::MatrixXf T = transformation.inverse() * eef_global * eefOffset;
    MathTools::eigen4f2rpy(T, d_w);

    Eigen::VectorXf target(6);
    for(int i = 0; i < 6; i++)
    {
        if(d_w[i] < bounds(i,0))
        {
            target(i) = bounds(i,0);
        }
        else if(d_w[i] > bounds(i,1))
        {
            target(i) = bounds(i,1);
        }
        else
        {
            target(i) = d_w[i];
        }
    }

    Eigen::Matrix4f T_dx;
    MathTools::posrpy2eigen4f(target.block<3,1>(0,0), target.block<3,1>(3,0), T_dx);

    float target_global[6];
    MathTools::eigen4f2rpy(transformation * T_dx, target_global);

    float e_global[6];
    MathTools::eigen4f2rpy(eef_global * eefOffset, e_global);

    resolveRPYAmbiguities(e_global, target_global);

    Eigen::VectorXf dx(6);
    dx << (target_global[0] - e_global[0]),
          (target_global[1] - e_global[1]),
          (target_global[2] - e_global[2]),
          getShortestDistanceForRPYComponent(e_global[3], target_global[3]),
          getShortestDistanceForRPYComponent(e_global[4], target_global[4]),
          getShortestDistanceForRPYComponent(e_global[5], target_global[5]);

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

const Eigen::Matrix4f &TSRConstraint::getTransformation()
{
    return transformation;
}

const Eigen::Matrix<float, 6, 2> &TSRConstraint::getBounds()
{
    return bounds;
}

void TSRConstraint::resolveRPYAmbiguities(float *pose, const float *reference)
{
    Eigen::Vector3f ref;
    ref << reference[3], reference[4], reference[5];

    Eigen::Vector3f tmp;

    Eigen::Vector3f best;
    best << pose[3], pose[4], pose[5];

    for(int i = -1; i <= 1; i += 2)
    {
        for(int j = -1; j <= 1; j += 2)
        {
            for(int k = -1; k <= 1; k += 2)
            {
                tmp << reference[3] + i * M_PI,
                       reference[4] + j * M_PI,
                       reference[5] + k * M_PI;

                if((tmp - ref).norm() < (best - ref).norm())
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

    if(direct > M_PI)
    {
        return -2*M_PI + direct;
    }
    else if(direct < -M_PI)
    {
        return 2*M_PI - direct;
    }
    else
    {
        return direct;
    }
}


