#include "GraspPlanner.h"

namespace GraspStudio
{
    GraspPlanner::GraspPlanner(VirtualRobot::GraspSetPtr graspSet)
        : graspSet(graspSet)
    {
        verbose = false;
    }

    GraspPlanner::~GraspPlanner()
    = default;

    void GraspPlanner::setVerbose(bool enable)
    {
        verbose = enable;
    }

    std::vector<VirtualRobot::GraspPtr> GraspPlanner::getPlannedGrasps()
    {
        return plannedGrasps;
    }

    GraspPlannerEvaluation GraspPlanner::getEvaluation()
    {
        return eval;
    }
}
