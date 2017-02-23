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

ConstrainedOptimizationIK::ConstrainedOptimizationIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, float timeout, float globalTolerance) :
    ConstrainedIK(robot, nodeSet, 30),
    nodeSet(nodeSet),
    timeout(timeout),
    globalTolerance(globalTolerance),
    functionValueTolerance(1e-6),
    optimizationValueTolerance(1e-4)
{
    setRandomSamplingDisplacementFactor(1);

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

    if(!std::isnan(globalTolerance))
    {
        optimizer->set_stopval(globalTolerance * globalTolerance);
    }

    optimizer->set_maxtime(timeout);
    optimizer->set_ftol_abs(functionValueTolerance);
    optimizer->set_xtol_abs(optimizationValueTolerance);

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
    return true;
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
    assert(maxIterations >= 0);
    for(unsigned int attempt = 0; attempt < static_cast<std::size_t>(maxIterations); attempt++)
    {
        numIterations = 0;

        //std::cout << "################################# New attempt: " << attempt << std::endl;

        int size = nodeSet->getSize();
        std::vector<double> x(size);

        if(attempt >= seeds.size())
        {
            // Try random configurations sampled around initial config
            for(int i = 0; i < size; i++)
            {
                float t = (rand()%1001) / 1000.0;
                x[i] = initialConfig(i) + randomSamplingDisplacementFactor * (nodeSet->getNode(i)->getJointLimitLo() + t * (nodeSet->getNode(i)->getJointLimitHi() - nodeSet->getNode(i)->getJointLimitLo()) - initialConfig(i));
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
                        x[i] = std::max(nodeSet->getNode(i)->getJointLimitLo(), std::min(nodeSet->getNode(i)->getJointLimitHi(), 0.0f));
                    }
                    break;

                case eSeedInitial:
                    // Try initial configuration
                    for(int i = 0; i < size; i++)
                    {
                        x[i] = std::max(nodeSet->getNode(i)->getJointLimitLo(), std::min(nodeSet->getNode(i)->getJointLimitHi(), initialConfig(i)));
                    }
                    break;

                case eSeedOther:
                    // Try used specified seed
                    Eigen::VectorXf s = seeds[attempt].second;
                    for(int i = 0; i < size; i++)
                    {
                        x[i] = std::max(nodeSet->getNode(i)->getJointLimitLo(), std::min(nodeSet->getNode(i)->getJointLimitHi(), s(i)));
                    }
                    break;
            }

            // Check initial configuration against joint limits
            for(unsigned int i = 0; i < nodeSet->getSize(); i++)
            {
                if(x[i] < nodeSet->getNode(i)->getJointLimitLo() || x[i] > nodeSet->getNode(i)->getJointLimitHi())
                {
                    THROW_VR_EXCEPTION("Initial configuration outside of joint limits: joints['" << nodeSet->getNode(i)->getName() << "'] = " << x[i] << ", Limits = [" << nodeSet->getNode(i)->getJointLimitLo() << ", " << nodeSet->getNode(i)->getJointLimitHi() << "]");
                }
            }
        }

        double min_f;

        try
        {
            /*nlopt::result result =*/ optimizer->optimize(x, min_f);
        }
        catch(const nlopt::roundoff_limited &/*e*/)
        {
            // This means that we optimize below the precision limit
            // The result might still be usable though
        }
        catch(const std::exception &e)
        {
            // This is something more severe, we still check the result and proceed
            // with the next attempt.
            VR_INFO << "Warning: NLOPT exception while optimizing: " << e.what() << std::endl;
        }

        for(int i = 0; i < size; i++)
        {
            nodeSet->getNode(i)->setJointValue(x[i]);
        }
        double currentError;
        bool success = hardOptimizationFunction(x, currentError);
        // We determine success based on hard constraints only
        if(success)
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

void ConstrainedOptimizationIK::setRandomSamplingDisplacementFactor(float displacementFactor)
{
    randomSamplingDisplacementFactor = displacementFactor;
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
                grad += function.constraint->optimizationGradient(function.id);
            }
        }
    }

    if(size > 0)
    {
        grad.normalize();

        for(unsigned int i = 0; i < gradient.size(); i++)
        {
            gradient[i] = grad(i);
        }
    }

    //std::cout << "optval: " << value << std::endl;

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

bool ConstrainedOptimizationIK::hardOptimizationFunction(const std::vector<double> &x, double & error)
{
    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }
    bool result = true;
    error = 0;
    for(auto &constraint : constraints)
    {
        for(auto &function : constraint->getOptimizationFunctions())
        {
            if(function.soft)
            {
                // Soft constraints do not count for hard optimization value
                continue;
            }

            result &= function.constraint->checkTolerances();
            error += function.constraint->optimizationFunction(function.id);
        }
    }

    return result;
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

