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

#include <nlopt.hpp>

using namespace VirtualRobot;

ConstrainedOptimizationIK::ConstrainedOptimizationIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, float timeout, float tolerance) :
    ConstrainedIK(robot, nodeSet, 30),
    nodeSet(nodeSet),
    timeout(timeout),
    tolerance(tolerance)
{
    clearSeeds();
    addSeed(eSeedInitial);
    addSeed(eSeedZero);
}

bool ConstrainedOptimizationIK::initialize()
{
    int size = nodeSet->getSize();
    nodeSet->getJointValues(initialConfig);

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
    robot->setUpdateCollisionModel(false);

    std::vector<double> bestJointValues;
    double currentMinError = std::numeric_limits<double>::max();
    for(unsigned int attempt = 0; attempt < maxIterations; attempt++)
    {
        numIterations = 0;

        int size = nodeSet->getSize();
        std::vector<double> x(size);

        if(attempt >= seeds.size())
        {
            // Try random configurations
            for(int i = 0; i < size; i++)
            {
                float t = (rand()%1000) / 1000.0;
                x[i] = nodeSet->getNode(i)->getJointLimitLo() + t * (nodeSet->getNode(i)->getJointLimitHi() - nodeSet->getNode(i)->getJointLimitLo());
            }
        }
        else
        {
            switch(seeds[attempt].first)
            {
                case eSeedZero:
                    // Try zero configuration
                    for(int i = 0; i < size; i++)
                    {
                        x[i] = 0;
                    }
                    break;

                case eSeedInitial:
                    // Try initial configuration
                    for(int i = 0; i < size; i++)
                    {
                        x[i] = initialConfig(i);
                    }
                    break;

                case eSeedOther:
                    // Try used specified seed
                    Eigen::VectorXf s = seeds[attempt].second;
                    for(int i = 0; i < size; i++)
                    {
                        x[i] = s(i);
                    }
                    break;
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
        double currentError = hardOptimizationFunction(x);
        // We determine success based on hard constraints only
        if(currentError < tolerance * tolerance)
        {
            // Success
            robot->setUpdateVisualization(updateVisualization);
            robot->setUpdateCollisionModel(updateCollisionModel);
            robot->updatePose(true);
            nodeSet->setJointValues(std::vector<float>(x.begin(), x.end()));
            return true;
        }
        else if(currentMinError > currentError)
        {
            currentMinError = currentError;
            bestJointValues = x;
        }

    }
    if(bestJointValues.size() > 0)
    {
        nodeSet->setJointValues(std::vector<float>(bestJointValues.begin(), bestJointValues.end()));
    }
    // Failure
    robot->setUpdateVisualization(updateVisualization);
    robot->setUpdateCollisionModel(updateCollisionModel);
    robot->updatePose(true);
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

double ConstrainedOptimizationIK::hardOptimizationFunction(const std::vector<double> &x)
{
    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }

    double value = 0;

    for(auto &constraint : constraints)
    {
        for(auto &function : constraint->getOptimizationFunctions())
        {
            if(function.soft)
            {
                // Soft constraints do not count for hard optimization value
                continue;
            }

            value += function.constraint->optimizationFunction(function.id);
        }
    }

    return value;
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

