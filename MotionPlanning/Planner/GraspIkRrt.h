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
* @package    MotionPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _MotionPlanning_GraspIkRrt_h
#define _MotionPlanning_GraspIkRrt_h

#include "../MotionPlanning.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/IK/AdvancedIKSolver.h>
#include "BiRrt.h"

namespace MotionPlanning
{

    /*!
     *
     * The GraspIkRrt planner combines the search for a feasible grasp and an IK solution with the search for a collision-free motion.
     *
     */
    class MOTIONPLANNING_IMPORT_EXPORT GraspIkRrt : public BiRrt
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param cspace The C-Space that should be used for collision detection
            \param object The object to be grasped
            \param ikSolver The ikSolver that should be used
            \param graspSet The grasps, defining potential goal configurations.
            \param probabSampleGoal Probability with which a goal config is created during planning loop.
        */
        GraspIkRrt(CSpaceSampledPtr cspace, VirtualRobot::ManipulationObjectPtr object, VirtualRobot::AdvancedIKSolverPtr ikSolver, VirtualRobot::GraspSetPtr graspSet, float probabSampleGoal = 0.1f);
        virtual ~GraspIkRrt();

        /*!
            do the planning (blocking method)
            \return true if solution was found, otherwise false
        */
        virtual bool plan(bool bQuiet = false);


        virtual void printConfig(bool printOnlyParams = false);

        //! This is not allowed here, since we sample goal configurations during planning: If called an exception is thrown
        virtual bool setGoal(const Eigen::VectorXf& c);

        //! reset the planner
        virtual void reset();

    protected:

        //virtual bool createSolution(bool bQuiet = false);
        bool doPlanningCycle();
        bool searchNewGoal();
        bool checkGoalConfig(Eigen::VectorXf& config);
        bool addIKSolution(Eigen::VectorXf& config, VirtualRobot::GraspPtr grasp);
        float sampleGoalProbab;

        VirtualRobot::ManipulationObjectPtr object;
        VirtualRobot::AdvancedIKSolverPtr ikSolver;
        VirtualRobot::GraspSetPtr graspSet;
        VirtualRobot::JointSetPtr rns;

        VirtualRobot::GraspSetPtr graspSetWorking;

        bool found; //!< Indicates if a solution was found

        std::map < VirtualRobot::GraspPtr, MotionPlanning::CSpaceNodePtr > graspNodeMapping;
        std::vector< Eigen::VectorXf > ikSolutions;

    };

} // namespace

#endif // _MotionPlanning_RRT_h


