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
#ifndef _VirtualRobot_Constraint_h_
#define _VirtualRobot_Constraint_h_

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/IK/JacobiProvider.h"

#include <boost/shared_ptr.hpp>

#include <nlopt.hpp>

class SoSeparator;

namespace VirtualRobot
{
    class Constraint;
    typedef double (*OptimizationFunction)(std::vector<double> &gradient);

    class OptimizationFunctionSetup
    {
        public:
            unsigned int id;
            Constraint *constraint;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT Constraint : public JacobiProvider, public boost::enable_shared_from_this<Constraint>
    {
    public:
        Constraint(const RobotNodeSetPtr& nodeSet);

        void initialize();

        virtual bool getRobotPoseForConstraint(Eigen::Matrix4f& pose);

        float getErrorDifference();

        virtual std::string getConstraintType() = 0;

        const std::vector<OptimizationFunctionSetup> &getEqualityConstraints();
        const std::vector<OptimizationFunctionSetup> &getInequalityConstraints();
        const std::vector<OptimizationFunctionSetup> &getOptimizationFunctions();

        virtual double optimizationFunction(unsigned int id) = 0;
        virtual Eigen::VectorXf optimizationGradient(unsigned int id) = 0;

    protected:
        void addEqualityConstraint(unsigned int id);
        void addInequalityConstraint(unsigned int id);
        void addOptimizationFunction(unsigned int id);

    protected:
        std::vector<OptimizationFunctionSetup> equalityConstraints;
        std::vector<OptimizationFunctionSetup> inequalityConstraints;
        std::vector<OptimizationFunctionSetup> optimizationFunctions;

        float lastError;
        float lastLastError;
    };

    typedef boost::shared_ptr<Constraint> ConstraintPtr;
}

#endif
