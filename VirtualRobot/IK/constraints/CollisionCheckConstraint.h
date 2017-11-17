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
#ifndef _VirtualRobot_COLLISIONCHECKCONSTRAINT_H
#define _VirtualRobot_COLLISIONCHECKCONSTRAINT_H

#include "../Constraint.h"




namespace VirtualRobot {

class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckConstraint : public Constraint
{
public:
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

#endif // COLLISIONCHECKCONSTRAINT_H
