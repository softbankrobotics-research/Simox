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
#ifndef _VirtualRobot_ConstrainedIK_h_
#define _VirtualRobot_ConstrainedIK_h_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/Model/Model.h"

#include <vector>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedIK
    {
    public:
        enum SeedType
        {
            // Zero configuration
            eSeedZero,

            // The configuration that was applied when initialize() was called
            eSeedInitial,

            // Some other configuration
            eSeedOther
        };

    public:
        ConstrainedIK(ModelPtr& robot, const JointSetPtr& nodeSet, int maxIterations = 1000, float stall_epsilon = 0.0001, float raise_epsilon = 0.8);// , bool reduceRobot = false);

        void addConstraint(const ConstraintPtr& constraint, int priority = 0, bool hard_constraint = true);
        void removeConstraint(const ConstraintPtr& constraint);
        std::vector<ConstraintPtr> getConstraints();

        void addSeed(SeedType type, const Eigen::VectorXf seed = Eigen::VectorXf());
        void clearSeeds();

        virtual bool initialize();

        virtual bool solveStep() = 0;
        virtual bool solve(bool stepwise = false);

        void setMaxIterations(int maxIterations);
        int getMaxIterations();

        bool getRunning();
        int getCurrentIteration();

    protected:
        //void getUnitableNodes(const RobotNodePtr &robotNode, const RobotNodeSetPtr &nodeSet, std::vector<std::string> &unitable);
        //RobotPtr buildReducedRobot(const RobotPtr &original);

    protected:
        std::vector<ConstraintPtr> constraints;
        std::map<ConstraintPtr, int> priorities;
        std::map<ConstraintPtr, bool> hardConstraints;

        RobotPtr originalRobot;
        RobotPtr robot;
        JointSetPtr nodeSet;
        Eigen::VectorXf initialConfig;

        std::vector<std::pair<SeedType, Eigen::VectorXf> > seeds;

        int maxIterations;
        int currentIteration;
        bool running;

        float stallEpsilon;
        float raiseEpsilon;
    };

    typedef std::shared_ptr<ConstrainedIK> ConstrainedIKPtr;
}

#endif
