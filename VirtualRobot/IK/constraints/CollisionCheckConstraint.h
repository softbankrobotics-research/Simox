/*
 * This file is part of ArmarX.
 *
 * Copyright (C) 2011-2017, High Performance Humanoid Technologies (H2T), Karlsruhe Institute of Technology (KIT), all rights reserved.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    ArmarX
 * @author     Mirko Waechter( mirko.waechter at kit dot edu)
 * @date       2017
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */
#pragma once

#include "../Constraint.h"




namespace VirtualRobot {

/**
 * @brief The CollisionCheckConstraint is a simple constraint for considering collisions during solving the inverse kinematics.
 *
 * This constraint does not provide a gradient. It only rejects invalid configurations in the function checkTolerances().
 */
class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckConstraint : public Constraint
{
public:
    /**
     * @brief CollisionCheckConstraint
     * @param rns RobotNodeSet used for the IK
     * @param cdm Already configured collision manager. Will be used for the collision check.
     */
    CollisionCheckConstraint(const VirtualRobot::RobotNodeSetPtr & rns, const VirtualRobot::CDManagerPtr &cdm);

    // JacobiProvider interface
    bool checkTolerances() override;

    // Constraint interface
    double optimizationFunction(unsigned int id) override;
    Eigen::VectorXf optimizationGradient(unsigned int id) override;
    bool usingCollisionModel() override;
protected:
    VirtualRobot::CDManagerPtr cdm;
    Eigen::VectorXf zeroVec;

};

} // namespace VirtualRobot

