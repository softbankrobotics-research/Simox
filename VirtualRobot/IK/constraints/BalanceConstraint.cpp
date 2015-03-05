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
                                     float tolerance, float minimumStability, float maxSupportDistance) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, contactNodes, tolerance, minimumStability, maxSupportDistance);
}

BalanceConstraint::BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SupportPolygonPtr &supportPolygon,
                                     float tolerance, float minimumStability, float maxSupportDistance) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, supportPolygon->getContactModels(), tolerance, minimumStability, maxSupportDistance);
}

void BalanceConstraint::initialize(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes, float tolerance, float minimumStability, float maxSupportDistance)
{
    this->joints = joints;
    this->bodies = bodies;
    this->minimumStability = minimumStability;

    supportPolygon.reset(new SupportPolygon(contactNodes));
    supportPolygon->updateSupportPolygon(maxSupportDistance);

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    THROW_VR_EXCEPTION_IF(!convexHull, "Empty support polygon for balance constraint");

    Eigen::Vector2f supportPolygonCenter = MathTools::getConvexHullCenter(convexHull);

    comIK.reset(new CoMIK(joints, bodies));
    comIK->setGoal(supportPolygonCenter, tolerance);

    initialized = true;
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
    return (supportPolygon->getStabilityIndex(bodies, false) >= minimumStability);
}

void BalanceConstraint::visualize(SoSeparator *sep)
{
    // The visualization assumes the support polygon to be horizontal, i.e. the normal of the floor plane
    // should point into z direction

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    MathTools::Plane floor = supportPolygon->getFloorPlane();

    if(!convexHull)
    {
        return;
    }

    SoSeparator *s1 = new SoSeparator;
    sep->addChild(s1);

    SoMaterial *m = new SoMaterial;
    m->diffuseColor.setValue(visualizationColor(0), visualizationColor(1), visualizationColor(2));
    m->ambientColor.setValue(visualizationColor(0), visualizationColor(1), visualizationColor(2));
    s1->addChild(m);

    SoDrawStyle *d = new SoDrawStyle;
    d->lineWidth.setValue(3);
    s1->addChild(d);

    SoCoordinate3 *coordinate = new SoCoordinate3;
    for (size_t i = 0; i < convexHull->segments.size(); i++)
    {
        int i1 = convexHull->segments[i].id1;
        int i2 = convexHull->segments[i].id2;

        if(i == 0)
        {
            coordinate->point.set1Value(i, convexHull->vertices[i1].x(), convexHull->vertices[i1].y(), floor.p.z());
        }

        coordinate->point.set1Value(i+1, convexHull->vertices[i2].x(), convexHull->vertices[i2].y(), floor.p.z());
    }
    s1->addChild(coordinate);

    SoLineSet *lineSet = new SoLineSet;
    s1->addChild(lineSet);

    SoSeparator *s2 = new SoSeparator;
    sep->addChild(s2);
    s2->addChild(m);

    Eigen::Vector2f center = MathTools::getConvexHullCenter(convexHull);
    SoTransform *t = new SoTransform;
    t->translation.setValue(center.x(), center.y(), floor.p.z());
    s2->addChild(t);

    SoSphere *s = new SoSphere;
    s->radius = 10;
    s2->addChild(s);
}

void BalanceConstraint::visualizeContinuously(SoSeparator *sep)
{
    SoSeparator *s3 = new SoSeparator;
    sep->addChild(s3);

    SoMaterial *m = new SoMaterial;
    m->diffuseColor.setValue(1, 0, 0);
    m->ambientColor.setValue(1, 0, 0);
    s3->addChild(m);

    Eigen::Vector3f com = bodies->getCoM();
    SoTransform *t = new SoTransform;
    t->translation.setValue(com(0), com(1), 0);
    s3->addChild(t);

    SoSphere *s = new SoSphere;
    s->radius = 10;
    s3->addChild(s);

    t = new SoTransform();
    t->translation.setValue(0, 0, com(2));
    s3->addChild(t);
    s3->addChild(s);
}

std::string BalanceConstraint::getConstraintType()
{
    return "Balance(" + joints->getName() + ")";
}


