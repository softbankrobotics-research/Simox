#include "BalanceConstraint.h"

#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>

using namespace VirtualRobot;

BalanceConstraint::BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, contactNodes, tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates);
}

BalanceConstraint::BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SupportPolygonPtr &supportPolygon,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, supportPolygon->getContactModels(), tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates);
}

void BalanceConstraint::initialize(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes, float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates)
{
    this->joints = joints;
    this->bodies = bodies;
    this->minimumStability = minimumStability;
    this->maxSupportDistance = maxSupportDistance;
    this->tolerance = tolerance;
    this->supportPolygonUpdates = supportPolygonUpdates;

    supportPolygon.reset(new SupportPolygon(contactNodes));

    comIK.reset(new CoMIK(joints, bodies));

    updateSupportPolygon();

    initialized = true;
}

void BalanceConstraint::updateSupportPolygon()
{
    supportPolygon->updateSupportPolygon(maxSupportDistance);

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    THROW_VR_EXCEPTION_IF(!convexHull, "Empty support polygon for balance constraint");

    Eigen::Vector2f supportPolygonCenter = MathTools::getConvexHullCenter(convexHull);
    comIK->setGoal(supportPolygonCenter, tolerance);
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix()
{
    return comIK->getJacobianMatrix();
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    return comIK->getJacobianMatrix(tcp);
}

Eigen::VectorXf BalanceConstraint::getError(float stepSize)
{
    if(supportPolygonUpdates)
    {
        updateSupportPolygon();
    }

    float stability = supportPolygon->getStabilityIndex(bodies, false);

    if(stability < minimumStability)
    {
        return comIK->getError(stepSize);
    }
    else
    {
        return Eigen::Vector2f(0,0);
    }
}

bool BalanceConstraint::checkTolerances()
{
    return (supportPolygon->getStabilityIndex(bodies, supportPolygonUpdates) >= minimumStability);
}

Eigen::Vector3f BalanceConstraint::getCoM()
{
    return bodies->getCoM();
}

SupportPolygonPtr BalanceConstraint::getSupportPolygon()
{
    return supportPolygon;
}

std::string BalanceConstraint::getConstraintType()
{
    return "Balance(" + joints->getName() + ")";
}


