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
* @package    GraspPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../GraspPlanning.h"

#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <GraspPlanning/GraspPlanner/GraspPlannerEvaluation.h>
#include <vector>

namespace GraspPlanning
{
    /*!
    *
    * \brief An interface for grasp planners.
    *
    */
    class GRASPPLANNING_IMPORT_EXPORT GraspPlanner
    {
    public:

        /*!
            Constructor
            \param graspSet Append planned grasps to this set.
        */
        GraspPlanner(const VirtualRobot::GraspSetPtr &graspSet);

        //! destructor
        virtual ~GraspPlanner();

        void setVerbose(bool enable);


        /*!
            Creates new grasps.
            \param nrGrasps The number of grasps to be planned.
            \param timeOutMS The time out in milliseconds. Planning is stopped when this time is exceeded. Disabled when zero.
            \return Number of planned grasps.
        */
        virtual int plan(int nrGrasps, int timeOutMS = 0, std::vector<VirtualRobot::ModelPtr> obstacles = std::vector<VirtualRobot::ModelPtr>()) = 0;

        /*!
            Returns all grasps that have been generated. These grasps are also stored in the graspSet that was specified on construction.
        */
        std::vector<VirtualRobot::GraspPtr> getPlannedGrasps();

        /*!
         * \brief getEvaluation
         * \return The current evaluation of the grasp planner.
         */
        GraspPlannerEvaluation getEvaluation();

    protected:
        bool verbose;
        VirtualRobot::GraspSetPtr graspSet;
        std::vector<VirtualRobot::GraspPtr> plannedGrasps;

        GraspPlannerEvaluation eval;
    };
}

