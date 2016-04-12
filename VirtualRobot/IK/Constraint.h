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

        const std::vector<OptimizationFunctionSetup> &getEqualityConstraints();
        const std::vector<OptimizationFunctionSetup> &getInequalityConstraints();
        const std::vector<OptimizationFunctionSetup> &getOptimizationFunctions();

        // Interface for NLopt-based solvers (default implementations)
        virtual double optimizationFunction(unsigned int id);
        virtual Eigen::VectorXf optimizationGradient(unsigned int id);

        // Interface for Jacobian-based solvers (default implementations)
        virtual Eigen::MatrixXf getJacobianMatrix();
        virtual Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp);
        virtual Eigen::VectorXf getError(float stepSize = 1.0f);
        virtual bool checkTolerances();

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
