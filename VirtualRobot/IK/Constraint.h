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

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/JacobiProvider.h"

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
            bool soft;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT Constraint : public JacobiProvider
    {
    public:
        Constraint(const JointSetPtr& nodeSet);

        void initialize();

        virtual bool getRobotPoseForConstraint(Eigen::Matrix4f& pose);

        float getErrorDifference();

        const std::vector<OptimizationFunctionSetup> &getEqualityConstraints();
        const std::vector<OptimizationFunctionSetup> &getInequalityConstraints();
        const std::vector<OptimizationFunctionSetup> &getOptimizationFunctions();

        /*!
         * Each constraint implements its own optimization function that contributes to a combined
         * optimization function of a constrained IK problem.
         *
         * The individual optimization function value of a constraint is multiplied by the given factor
         * in order to weigh the contribution of the individual constraint.
         *
         * The default value of this factor is 1.
         */
        void setOptimizationFunctionFactor(float factor);
        float getOptimizationFunctionFactor();

        // Interface for NLopt-based solvers (default implementations)
        virtual double optimizationFunction(unsigned int id);
        virtual Eigen::VectorXf optimizationGradient(unsigned int id);

        // Interface for Jacobian-based solvers (default implementations)
        virtual Eigen::MatrixXf getJacobianMatrix() override;
        virtual Eigen::MatrixXf getJacobianMatrix(const FramePtr &tcp) override;
        virtual Eigen::VectorXf getError(float stepSize = 1.0f) override;
        virtual bool checkTolerances() override;

        virtual VisualizationPtr getVisualization() const;

    protected:
        void addEqualityConstraint(unsigned int id, bool soft=false);
        void addInequalityConstraint(unsigned int id, bool soft=false);
        void addOptimizationFunction(unsigned int id, bool soft=false);

    protected:
        std::vector<OptimizationFunctionSetup> equalityConstraints;
        std::vector<OptimizationFunctionSetup> inequalityConstraints;
        std::vector<OptimizationFunctionSetup> optimizationFunctions;

        float lastError;
        float lastLastError;

        float optimizationFunctionFactor;
    };

    typedef std::shared_ptr<Constraint> ConstraintPtr;
}

#endif
