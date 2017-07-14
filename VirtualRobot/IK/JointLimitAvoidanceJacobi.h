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
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_JointLimitAvoidanceJacobi_h_
#define _VirtualRobot_JointLimitAvoidanceJacobi_h_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/JacobiProvider.h"
#include "VirtualRobot/Model/JointSet.h"


namespace VirtualRobot
{

    /*!
            This class creates a dummy Jacobian matrix (identity) that can be used to specify a joint space task (e.g. a joint limit avoidance task)
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT JointLimitAvoidanceJacobi : public VirtualRobot::JacobiProvider
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        JointLimitAvoidanceJacobi(JointSetPtr rns, JacobiProvider::InverseJacobiMethod invJacMethod = JacobiProvider::eSVD);

        virtual Eigen::MatrixXf getJacobianMatrix();
        virtual Eigen::MatrixXf getJacobianMatrix(VirtualRobot::CoordinatePtr tcp);

        /*!
            Computes the complete error vector that is given by the distance to the center of the joint limits.
            Translational Joints are ignored (error = 0)
            \param stepSize The error can be reduced by this factor.
        */
        virtual Eigen::VectorXf getError(float stepSize = 1.0f);

        /*!
         *  Not used, only implemented because of superclass JacobiProvider, always returns true
         */
        bool checkTolerances();

    protected:
        std::vector<ModelJointPtr> nodes;
    };

    typedef std::shared_ptr<JointLimitAvoidanceJacobi> JointLimitAvoidanceJacobiPtr;

} // namespace VirtualRobot

#endif

