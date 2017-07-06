#include "Coordinate.h"


namespace VirtualRobot
{

Coordinate::Coordinate(const std::string &name)
    : globalPose(Eigen::Matrix4f::Identity()), name(name)
{
}

Coordinate::~Coordinate()
{
}

Eigen::Matrix4f Coordinate::getGlobalPose() const
{
    return globalPose;
}

Eigen::Vector3f Coordinate::getGlobalPosition() const
{
    return globalPose.block<3, 1>(0, 3);
}

std::string Coordinate::getName()
{
    return name;
}

} //namespace VirtualRobot

