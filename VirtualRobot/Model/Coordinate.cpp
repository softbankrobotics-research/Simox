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

void Coordinate::setName(const std::string & name)
{
    this->name = name;
}

Eigen::Matrix4f Coordinate::toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const
{
	return globalPose.inverse() * poseGlobal;
}

Eigen::Vector3f Coordinate::toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0, 3, 3, 1) = positionGlobal;
	t = toLocalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0, 3, 3, 1);
	return result;
}

Eigen::Matrix4f Coordinate::toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const
{
	return globalPose * poseLocal;
}

Eigen::Vector3f Coordinate::toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0, 3, 3, 1) = positionLocal;
	t = toGlobalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0, 3, 3, 1);
	return result;
}

Eigen::Matrix4f Coordinate::getTransformationTo(const CoordinatePtr& otherObject)
{
	return getGlobalPose().inverse() * otherObject->getGlobalPose();
}

Eigen::Matrix4f Coordinate::getTransformationFrom(const CoordinatePtr& otherObject)
{
	return otherObject->getGlobalPose().inverse() * getGlobalPose();
}

} //namespace VirtualRobot

