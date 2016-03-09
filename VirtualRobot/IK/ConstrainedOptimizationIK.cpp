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
    tolerance(tolerance)
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
    optimizer->set_stopval(tolerance);
    //optimizer->set_ftol_abs(1e-6);
    //optimizer->set_xtol_abs(1e-6);

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

    int size = nodeSet->getSize();
    std::vector<double> iv(size);
    for(int i = 0; i < size; i++)
    {
        iv[i] = nodeSet->getNode(i)->getJointValue();
    }

    double min_f;

    try
    {
        nlopt::result result = optimizer->optimize(iv, min_f);
    }
    catch(const nlopt::roundoff_limited &e)
    {
        // This means that we optimize below the precision limit
        // The result might still be usable though
    }
    catch(const std::exception &e)
    {
        throw;
        THROW_VR_EXCEPTION("NLOPT exception while optimizing!");
    }

    std::cout << "Solution: " << std::endl;
    unsigned int i = 0;
    for(auto &constraint : constraints)
    {
        auto eq = constraint->getEqualityConstraints();
        auto ineq = constraint->getInequalityConstraints();

        for(auto &c : eq)
        {
            std::cout << "    - Eq. Constraint " << i << ": " << c.constraint->optimizationFunction(c.id) << std::endl;
            i++;
        }

        for(auto &c : ineq)
        {
            std::cout << "    - Ineq. Constraint " << i << ": " << c.constraint->optimizationFunction(c.id) << std::endl;
            i++;
        }
    }

}

bool ConstrainedOptimizationIK::solveStep()
{
    THROW_VR_EXCEPTION("Stepwise solving not possible with optimization IK");
}

double ConstrainedOptimizationIK::optimizationFunction(const std::vector<double> &x, std::vector<double> &gradient)
{
    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }

    unsigned int size = gradient.size();
    if(size > 0)
    {
        for(unsigned int i = 0; i < size; i++)
        {
            gradient[i] = 0;
        }
    }

    double value = 0;

    for(auto &constraint : constraints)
    {
        for(auto &function : constraint->getOptimizationFunctions())
        {
            value += function.constraint->optimizationFunction(function.id);
            function.constraint->optimizationGradient(function.id, gradient);
        }
    }

    //std::cout << "Optimization value: " << value << std::endl;

    return value;
}

double ConstrainedOptimizationIK::optimizationConstraint(const std::vector<double> &x, std::vector<double> &gradient, const OptimizationFunctionSetup &setup)
{
    if(x != currentX)
    {
        std::vector<float> q(x.begin(), x.end());
        nodeSet->setJointValues(q);
        currentX = x;
    }

    if(gradient.size() > 0)
    {
        setup.constraint->optimizationGradient(setup.id, gradient);
    }

    float value = setup.constraint->optimizationFunction(setup.id);

    std::cout << setup.id << ": Opt. value: " << value << std::endl;
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

