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

#include "BalanceConstraint.h"

#include <VirtualRobot/Tools/MathTools.h>

using namespace VirtualRobot;

BalanceConstraintOptimizationFunction::BalanceConstraintOptimizationFunction(const SupportPolygonPtr &supportPolygon) :
    supportPolygon(supportPolygon)
{
    update();
}

void BalanceConstraintOptimizationFunction::updateSupportPolygon()
{
    supportPolygon->updateSupportPolygon();
    update();
}

double BalanceConstraintOptimizationFunction::evaluateOptimizationFunction(const Eigen::Vector2f &com)
{
    double result = 1;

    unsigned int numHalfspaces = matrices.size();
    for(unsigned int i = 0; i < numHalfspaces; i++)
    {
        Eigen::Matrix2f R = matrices[i];
        Eigen::Vector2f d = displacements[i];

        result *= sigmoid((R * (com - d))[0]);
    }

    return 1 - result;
}

Eigen::VectorXf BalanceConstraintOptimizationFunction::evaluateOptimizationGradient(const Eigen::Vector2f &com, const Eigen::MatrixXf &Jcom)
{
    unsigned int numHalfspaces = matrices.size();

    std::vector<double> f_i(numHalfspaces);
    std::vector<Eigen::VectorXf> f_prime_i(numHalfspaces);

    for(unsigned int i = 0; i < numHalfspaces; i++)
    {
        Eigen::Matrix2f R = matrices[i];
        Eigen::Vector2f d = displacements[i];

        float v = (R * (com - d))[0];

        f_i[i] = sigmoid(v);
        f_prime_i[i] = (R * Jcom).row(0) * float(sigmoid_prime(v));
    }

    Eigen::VectorXf result(Jcom.cols());
    result.setZero();

    for(unsigned int i = 0; i < numHalfspaces; i++)
    {
        double prod = 1;
        for(unsigned int k = 0; k < numHalfspaces; k++)
        {
            if(k == i)
            {
                continue;
            }

            prod *= f_i[k];
        }

        result += f_prime_i[i] * float(prod);
    }

    return -result;
}

void BalanceConstraintOptimizationFunction::update()
{
    matrices.clear();
    displacements.clear();

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    if(!convexHull)
    {
        return;
    }

    for(MathTools::Segment2D &segment : convexHull->segments)
    {
        Eigen::Vector2f p1 = convexHull->vertices[segment.id1];
        Eigen::Vector2f p2 = convexHull->vertices[segment.id2];

        displacements.push_back(p1);

        Eigen::Vector2f v = (p2 - p1).normalized();

        Eigen::Matrix2f R;
        R << -v[1], v[0],
              v[0], v[1];
        matrices.push_back(R.transpose());
    }
}

double BalanceConstraintOptimizationFunction::sigmoid(double x)
{
    double beta = 0.1;
    return 1.0 / (1.0 + exp(-beta * x));
}

double BalanceConstraintOptimizationFunction::sigmoid_prime(double x)
{
    return sigmoid(x) * (1 - sigmoid(x));
}


BalanceConstraint::BalanceConstraint(const ModelPtr& robot, const JointSetPtr& joints, const LinkSetPtr& bodies, const LinkSetPtr& contactNodes,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, contactNodes, tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates, considerCoMHeight);
}

BalanceConstraint::BalanceConstraint(const ModelPtr& robot, const JointSetPtr& joints, const LinkSetPtr& bodies, const SupportPolygonPtr& supportPolygon,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, supportPolygon->getContactModels(), tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates, considerCoMHeight);
}

void BalanceConstraint::initialize(const ModelPtr& robot, const JointSetPtr& joints, const LinkSetPtr& bodies, const LinkSetPtr& contactNodes,
                                   float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight)
{
    this->joints = joints;
    this->bodies = bodies;
    this->minimumStability = minimumStability;
    this->maxSupportDistance = maxSupportDistance;
    this->tolerance = tolerance;
    this->supportPolygonUpdates = supportPolygonUpdates;
    this->considerCoMHeight = considerCoMHeight;

    supportPolygon.reset(new SupportPolygon(contactNodes));
    differnentiableStability.reset(new BalanceConstraintOptimizationFunction(supportPolygon));

    VR_INFO << "COM height:" << height << endl;
    int dim = 2;

    if (considerCoMHeight)
    {
        dim = 3;
        setCoMHeight(height);
    }
    else
    {
        height = robot->getCoMGlobal()(2);
        setCoMHeight(height);
    }

    comIK.reset(new CoMIK(joints, bodies, RobotNodePtr(), dim));

    updateSupportPolygon();

    addOptimizationFunction(0);

    initialized = true;
}

void BalanceConstraint::updateSupportPolygon()
{
    supportPolygon->updateSupportPolygon(maxSupportDistance);

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    THROW_VR_EXCEPTION_IF(!convexHull, "Empty support polygon for balance constraint");

    differnentiableStability->updateSupportPolygon();

    Eigen::Vector2f supportPolygonCenter = MathTools::getConvexHullCenter(convexHull);

    if (considerCoMHeight)
    {
        Eigen::Vector3f goal;
        goal.head(2) = supportPolygonCenter;
        goal(2) = height;
        comIK->setGoal(goal, tolerance);
        //VR_INFO << "CoM Height of Inversed Robot set to:" << goal(2) << endl;
    }
    else
    {
        comIK->setGoal(supportPolygonCenter, tolerance);
    }
}

float BalanceConstraint::getDifferentiableStabilityIndex()
{
    if(supportPolygonUpdates)
    {
        supportPolygon->updateSupportPolygon();
    }

    MathTools::ConvexHull2DPtr ch = supportPolygon->getSupportPolygon2D();
    Eigen::Vector2f com2D = bodies->getCoM().head(2);

    if (!ch || ch->vertices.size() < 2 || !MathTools::isInside(com2D, ch))
    {
        return 0;
    }

    return 0;
}

Eigen::VectorXf BalanceConstraint::getDifferentiableStabilityIndexGradient()
{
    Eigen::VectorXf result;
    return result;
}

void BalanceConstraint::setCoMHeight(float currentheight)
{
    height = currentheight;
}

double BalanceConstraint::optimizationFunction(unsigned int /*id*/)
{
    if(supportPolygonUpdates)
    {
        differnentiableStability->updateSupportPolygon();
    }

    Eigen::Vector2f com = bodies->getCoM().head(2);
    if(considerCoMHeight)
    {
        VR_WARNING << "COM Height not yet supported by NLopt balance constraint" << std::endl;
    }

    return differnentiableStability->evaluateOptimizationFunction(com);
}

Eigen::VectorXf BalanceConstraint::optimizationGradient(unsigned int /*id*/)
{
    if(supportPolygonUpdates)
    {
        differnentiableStability->updateSupportPolygon();
    }

    Eigen::Vector2f com = bodies->getCoM().head(2);
    if(considerCoMHeight)
    {
        VR_WARNING << "COM Height not yet supported by NLopt balance constraint" << std::endl;
    }

    return differnentiableStability->evaluateOptimizationGradient(com, comIK->getJacobianMatrix());
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix()
{
    return comIK->getJacobianMatrix();
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix(FramePtr tcp)
{
    return comIK->getJacobianMatrix(tcp);
}

Eigen::VectorXf BalanceConstraint::getError(float stepSize)
{
    float stability = supportPolygon->getStabilityIndex(bodies, supportPolygonUpdates);

    if (supportPolygonUpdates)
    {
        updateSupportPolygon();
    }

    if (stability < minimumStability)
    {
        return comIK->getError(stepSize);
    }
    else if (considerCoMHeight)
    {
        return Eigen::Vector3f(0, 0, 0);
    }
    else
    {
        return Eigen::Vector2f(0, 0);
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

