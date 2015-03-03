#include "Constraint.h"

using namespace VirtualRobot;

Constraint::Constraint(const RobotNodeSetPtr &nodeSet) :
    JacobiProvider(nodeSet, JacobiProvider::eSVD)
{

}

bool Constraint::getRobotPoseForConstraint(Eigen::Matrix4f &pose)
{
    // No change in global pose required
    return false;
}

void Constraint::visualize(SoSeparator *sep)
{
}

void Constraint::visualizeContinuously(SoSeparator *sep)
{
}

void Constraint::setVisualizationColor(const Eigen::Vector4f &color)
{
    visualizationColor = color;
}


