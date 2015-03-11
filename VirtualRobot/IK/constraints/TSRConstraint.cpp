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
    MathTools::eigen4f2rpy(eef->getGlobalPose(), eef_pose);
    MathTools::eigen4f2rpy(transformation * eefOffset, target);

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

void TSRConstraint::visualize(SoSeparator *sep)
{
    // TODO: Visualization only works for symmetric bounds

    // Use full transparency as visualization switch
    if(visualizationColor(3) == 1)
    {
        return;
    }

    SoSeparator *s = new SoSeparator;
    sep->addChild(s);

    SoMaterial *m = new SoMaterial;
    m->diffuseColor.setValue(visualizationColor(0), visualizationColor(1), visualizationColor(2));
    m->ambientColor.setValue(visualizationColor(0), visualizationColor(1), visualizationColor(2));
    m->transparency.setValue(visualizationColor(3));
    s->addChild(m);

    SoTransform *t = new SoTransform;
    t->translation.setValue(transformation(0,3), transformation(1,3), transformation(2,3));
    MathTools::Quaternion q = MathTools::eigen4f2quat(transformation);
    t->rotation.setValue(q.x, q.y, q.z, q.w);
    s->addChild(t);

    SoCube *c = new SoCube;
    c->width = fabs(bounds(0,0) - bounds(0,1));
    c->height = fabs(bounds(1,0) - bounds(1,1));
    c->depth = fabs(bounds(2,0) - bounds(2,1));
    s->addChild(c);
}


