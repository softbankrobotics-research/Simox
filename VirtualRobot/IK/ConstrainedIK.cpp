#include "ConstrainedIK.h"

using namespace VirtualRobot;

ConstrainedIK::ConstrainedIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet, int maxIterations, float stall_epsilon, float raise_epsilon, bool reduceRobot) :
    originalRobot(robot),
    nodeSet(nodeSet),
    maxIterations(maxIterations),
    currentIteration(0),
    running(false),
    stallEpsilon(stall_epsilon),
    raiseEpsilon(raise_epsilon)
{
    addSeed(eSeedInitial);

    // Reduce the provided robot to only those nodes requested for the IK. This has the advantage that
    // joints not necessary for the IK (e.g. finger joints) are not updated
    if(reduceRobot)
    {
        VR_WARNING << "Robot reduction is EXPERIMENTAL" << std::endl;
        this->robot = buildReducedRobot(originalRobot);
    }
    else
    {
        this->robot = originalRobot;
    }
}

void ConstrainedIK::addConstraint(const ConstraintPtr& constraint, int priority, bool hard_constraint)
{
    constraints.push_back(constraint);
    priorities[constraint] = priority;
    hardConstraints[constraint] = hard_constraint;
    std::sort(constraints.begin(), constraints.end(), [this](const ConstraintPtr & lhs, const ConstraintPtr & rhs)
    {
        return priorities[lhs] > priorities[rhs];
    });
}

void ConstrainedIK::removeConstraint(const ConstraintPtr& constraint)
{
    auto position = std::find(constraints.begin(), constraints.end(), constraint);

    if (position != constraints.end())
    {
        constraints.erase(position);
        priorities.erase(constraint);
        hardConstraints.erase(constraint);
    }
}

std::vector<ConstraintPtr> ConstrainedIK::getConstraints()
{
    return constraints;
}

void ConstrainedIK::addSeed(ConstrainedIK::SeedType type, const Eigen::VectorXf seed)
{
    seeds.push_back(std::pair<SeedType, Eigen::VectorXf>(type, seed));
}

void ConstrainedIK::clearSeeds()
{
    seeds.clear();
}

bool ConstrainedIK::initialize()
{
    Eigen::Matrix4f P;
    int moves = 0;

    nodeSet->getJointValues(initialConfig);

    for (auto & constraint : constraints)
    {
        constraint->initialize();
        moves += constraint->getRobotPoseForConstraint(P);
    }

    if (moves == 1)
    {
        // Only one constraint requested to move the robot => No conflicts
        robot->setGlobalPose(P);
    }
    else if (moves > 1)
    {
        VR_ERROR << "Multiple constraints requested to move the robot" << std::endl;
        return false;
    }

    running = true;
    currentIteration = 0;
    return true;
}

bool ConstrainedIK::solve(bool stepwise)
{
    if(seeds.size() != 1)
    {
        // TODO: Implement seeds for IKs other that ConstrainedOptimizationIK
        VR_WARNING << "Seeds not supported by this IK implementation" << std::endl;
    }

    while (currentIteration < maxIterations)
    {
        if (!solveStep())
        {
            running = false;
            return false;
        }

        bool goalReached = true;

        for (auto & constraint : constraints)
        {
            if (hardConstraints[constraint] && !constraint->checkTolerances())
            {
                goalReached = false;
            }
        }

        if (goalReached)
        {
            running = false;
            return true;
        }

        currentIteration++;

        if (stepwise)
        {
            // Identify this case via running variable
            return false;
        }
    }

    running = false;
    return false;
}

void ConstrainedIK::setMaxIterations(int maxIterations)
{
    this->maxIterations = maxIterations;
}

int ConstrainedIK::getMaxIterations()
{
    return maxIterations;
}

bool ConstrainedIK::getRunning()
{
    return running;
}

int ConstrainedIK::getCurrentIteration()
{
    return currentIteration;
}

void ConstrainedIK::getUnitableNodes(const RobotNodePtr &robotNode, const RobotNodeSetPtr &nodeSet, std::vector<std::string> &unitable)
{
    std::vector<RobotNodePtr> descendants;
    robotNode->collectAllRobotNodes(descendants);

    for (std::vector<RobotNodePtr>::const_iterator i = descendants.begin(); i != descendants.end(); i++)
    {
        // Skip current robot node
        if (*i == robotNode)
        {
            continue;
        }

        if(nodeSet->hasRobotNode(*i) || *i == nodeSet->getTCP())
        {
            // Actuated joint in subtree -> recursive descent
            std::vector<SceneObjectPtr> children = robotNode->getChildren();
            for (auto &child : children)
            {
                RobotNodePtr childRobotNode = boost::dynamic_pointer_cast<RobotNode>(child);
                if (childRobotNode)
                {
                    getUnitableNodes(childRobotNode, nodeSet, unitable);
                }
            }

            return;
        }
    }

    // No actuated joint in subtree
    if (!robotNode->getChildren().empty())
    {
        unitable.push_back(robotNode->getName());
    }
}

RobotPtr ConstrainedIK::buildReducedRobot(const RobotPtr &original)
{
    std::vector<std::string> names;
    for(int i = 0; i < nodeSet->getSize(); i++)
    {
        names.push_back(nodeSet->getNode(i)->getName());
    }

    std::vector<std::string> unitable;
    getUnitableNodes(original->getRootNode(), nodeSet, unitable);

    VR_INFO << "Reducing robot model by uniting " << unitable.size() << " nodes:" << std::endl;
    for(auto &node : unitable)
    {
        VR_INFO << "    " << node << std::endl;
    }

    return RobotFactory::cloneUniteSubsets(original, "ConstrainedIK_Reduced_Robot", unitable);
}


