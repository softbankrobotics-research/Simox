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
* @author     Matthias Hadlich
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "ConstrainedOptimizationIK.h"

using namespace VirtualRobot;

ConstrainedOptimizationIK::ConstrainedOptimizationIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, float timeout, float tolerance) :
    ConstrainedIK(robot),
    nodeSet(nodeSet),
    timeout(timeout),
    tolerance(tolerance),
    maxAttempts(30)
{
}

bool ConstrainedOptimizationIK::initialize()
{
    int size = nodeSet->getSize();

    optimizer.reset(new nlopt::opt(nlopt::LD_SLSQP, size));

    std::vector<double> low(size);
    std::vector<double> high(size);

    for(int i = 0; i < size; i++)
    {
        low[i] = nodeSet->getNode(i)->getJointLimitLo();
        high[i] = nodeSet->getNode(i)->getJointLimitHi();
    }

    optimizer->set_lower_bounds(low);
    optimizer->set_upper_bounds(high);

    optimizer->set_maxtime(timeout);
    optimizer->set_stopval(tolerance * tolerance);
    optimizer->set_ftol_abs(1e-6);
    optimizer->set_xtol_abs(1e-4);

    optimizer->set_min_objective(optimizationFunctionWrapper, this);

    for(auto &constraint : constraints)
    {
        for(auto &c : constraint->getEqualityConstraints())
        {
            optimizer->add_equality_constraint(optimizationConstraintWrapper, new std::pair<OptimizationFunctionSetup, ConstrainedOptimizationIK *>(c, this), 1e-6);
        }

        for(auto &c : constraint->getInequalityConstraints())
        {
            optimizer->add_inequality_constraint(optimizationConstraintWrapper, new std::pair<OptimizationFunctionSetup, ConstrainedOptimizationIK *>(c, this), 1e-6);
        }
    }
}

bool ConstrainedOptimizationIK::solve(bool stepwise)
{
    THROW_VR_EXCEPTION_IF(stepwise, "Stepwise solving not possible with optimization IK");
    THROW_VR_EXCEPTION_IF(!optimizer, "IK not initialized, did you forget to call initialize()?");

    bool updateVisualization = robot->getUpdateVisualizationStatus();
    bool updateCollisionModel = robot->getUpdateCollisionModelStatus();

    robot->setUpdateVisualization(false);
    robot->setUpdateVisualization(false);

    for(unsigned int attempt = 0; attempt < maxAttempts; attempt++)
    {
        numIterations = 0;

        int size = nodeSet->getSize();
        std::vector<double> x(size);
        for(int i = 0; i < size; i++)
        {
            if(attempt == 0)
            {
                // First try zero-position
                x[i] = 0;
            }
            else
            {
                // If zero position fails, try random positions
                float t = (rand()%1000) / 1000.0;
                x[i] = nodeSet->getNode(i)->getJointLimitLo() + t * (nodeSet->getNode(i)->getJointLimitHi() - nodeSet->getNode(i)->getJointLimitLo());
            }
        }

        double min_f;

        try
        {
            nlopt::result result = optimizer->optimize(x, min_f);
        }
        catch(const nlopt::roundoff_limited &e)
        {
            // This means that we optimize below the precision limit
            // The result might still be usable though
        }
        catch(const std::exception &e)
        {
            // This is something more severe, we still check the result and proceed
            // with the next attempt.
            VR_INFO << "Warning: NLOPT exception while optimizing" << std::endl;
        }

        for(int i = 0; i < size; i++)
        {
            nodeSet->getNode(i)->setJointValue(x[i]);
        }

        if(min_f < tolerance * tolerance)
        {
            // Success
            robot->setUpdateVisualization(updateVisualization);
            robot->setUpdateCollisionModel(updateCollisionModel);
            return true;
        }
    }

    // Failure
    robot->setUpdateVisualization(updateVisualization);
    robot->setUpdateCollisionModel(updateCollisionModel);
    return false;
}

bool ConstrainedOptimizationIK::solveStep()
{
    THROW_VR_EXCEPTION("Stepwise solving not possible with optimization IK");
}

double ConstrainedOptimizationIK::optimizationFunction(const std::vector<double> &x, std::vector<double> &gradient)
{
    numIterations++;

    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }

    unsigned int size = gradient.size();
    Eigen::VectorXf grad = Eigen::VectorXf::Zero(size);
    double value = 0;

    for(auto &constraint : constraints)
    {
        for(auto &function : constraint->getOptimizationFunctions())
        {
            value += function.constraint->optimizationFunction(function.id);

            if(size > 0)
            {
                Eigen::VectorXf g = function.constraint->optimizationGradient(function.id);

                // Crop gradients to a maximum size
                float n = g.norm();
                if(n > 1)
                {
                    g /= n;
                }

                grad += g;
            }
        }
    }

    if(size > 0)
    {
        for(unsigned int i = 0; i < gradient.size(); i++)
        {
            gradient[i] = grad(i);
        }
    }

    return value;
}

double ConstrainedOptimizationIK::optimizationConstraint(const std::vector<double> &x, std::vector<double> &gradient, const OptimizationFunctionSetup &setup)
{
    numIterations++;

    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }

    if(gradient.size() > 0)
    {
        Eigen::VectorXf g = setup.constraint->optimizationGradient(setup.id);

        for(unsigned int i = 0; i < gradient.size(); i++)
        {
            gradient[i] = g(i);
        }
    }

    return setup.constraint->optimizationFunction(setup.id);
}

double ConstrainedOptimizationIK::optimizationFunctionWrapper(const std::vector<double> &x, std::vector<double> &gradient, void *data)
{
    return static_cast<ConstrainedOptimizationIK *>(data)->optimizationFunction(x, gradient);
}

double ConstrainedOptimizationIK::optimizationConstraintWrapper(const std::vector<double> &x, std::vector<double> &gradient, void *data)
{
    return (static_cast<std::pair<OptimizationFunctionSetup, ConstrainedOptimizationIK *> *>(data)->second)->optimizationConstraint(
                x, gradient, static_cast<std::pair<OptimizationFunctionSetup, ConstrainedOptimizationIK *> *>(data)->first);
}

