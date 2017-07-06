#include "Cordinate.cpp"


namespace VirtualRobot
{

Coordinate::Coordinate()
    : globalPose(Eigen::Matrix4f::Identity())
{
}

Coordinate::~Coordinate()
{
}

Eigen::Matrix4f Coordinate::getGlobalPose() const
{
    return gloablPose;
}

Eigen::Vector3f Coordinate::getPosition() const
{
    return globalPose.block<3, 1>(0, 3);
}

} //namespace VirtualRobot

