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
#include "CollisionCheckConstraint.h"
#include "../../CollisionDetection/CDManager.h"

namespace VirtualRobot {

CollisionCheckConstraint::CollisionCheckConstraint(const JointSetPtr &joints, const CDManagerPtr &cdm) :
    Constraint(joints),
    cdm(cdm),
    zeroVec(Eigen::VectorXf::Zero(joints->getSize()))
{
    for (int i = 0; i < joints->getSize(); ++i) {
        zeroVec(i) = 1e-10;
    }
    addOptimizationFunction(0, false);
}



bool CollisionCheckConstraint::checkTolerances()
{
    bool result = !cdm->isInCollision();
    return result;
}

double CollisionCheckConstraint::optimizationFunction(unsigned int id)
{
    return 0;
}

Eigen::VectorXf CollisionCheckConstraint::optimizationGradient(unsigned int id)
{
    return zeroVec;
}


bool CollisionCheckConstraint::usingCollisionModel()
{
    return true;
}

}
