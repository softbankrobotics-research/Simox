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
    optimizer->set_ftol_rel(1e-6);
    optimizer->set_xtol_rel(1e-6);

    optimizer->set_min_objective(optimizationFunctionWrapper, this);
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
        THROW_VR_EXCEPTION("NLOPT exception while optimizing!");
    }
}

bool ConstrainedOptimizationIK::solveStep()
{
    THROW_VR_EXCEPTION("Stepwise solving not possible with optimization IK");
}

double ConstrainedOptimizationIK::optimizationFunction(const std::vector<double> &x, std::vector<double> &gradient, void *data)
{
    std::vector<float> q(x.begin(), x.end());
    nodeSet->setJointValues(q);

    bool useGradient = (gradient.size() > 0);
    Eigen::VectorXf gradientSum;

    if(useGradient)
    {
        gradientSum.resize(gradient.size());
        gradientSum.setZero();
    }

    float result = 0;
    for(auto &constraint : constraints)
    {
        result += constraint->optimizationFunction();

        if(useGradient)
        {
            gradientSum += constraint->optimizationGradient();
        }
    }

    if(useGradient)
    {
        for(unsigned int i = 0; i < gradient.size(); i++)
        {
            gradient[i] = gradientSum(i);
        }
    }

    return result;
}

double ConstrainedOptimizationIK::optimizationFunctionWrapper(const std::vector<double> &x, std::vector<double> &gradient, void *data)
{
    return static_cast<ConstrainedOptimizationIK *>(data)->optimizationFunction(x, gradient, NULL);
}

