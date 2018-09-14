#include "Frame.h"

#include "Model.h"

namespace VirtualRobot
{

Frame::Frame(const std::string &name)
    : globalPose(Eigen::Matrix4f::Identity()), name(name)
{
}

Frame::~Frame()
{
}

Eigen::Matrix4f Frame::getGlobalPose() const
{
    return globalPose;
}

Eigen::Vector3f Frame::getGlobalPosition() const
{
    return globalPose.block<3, 1>(0, 3);
}

std::string Frame::getName() const
{
    return name;
}

void Frame::setName(const std::string & name)
{
    this->name = name;
}

Eigen::Matrix4f Frame::toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const
{
	return globalPose.inverse() * poseGlobal;
}

Eigen::Vector3f Frame::toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0, 3, 3, 1) = positionGlobal;
	t = toLocalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0, 3, 3, 1);
	return result;
}

Eigen::Matrix4f Frame::toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const
{
	return globalPose * poseLocal;
}

Eigen::Vector3f Frame::toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0, 3, 3, 1) = positionLocal;
	t = toGlobalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0, 3, 3, 1);
	return result;
}

Eigen::Matrix4f Frame::getTransformationTo(const FramePtr& otherObject) const
{
	return getGlobalPose().inverse() * otherObject->getGlobalPose();
}

Eigen::Matrix4f Frame::getTransformationFrom(const FramePtr& otherObject) const
{
    return otherObject->getGlobalPose().inverse() * getGlobalPose();
}

Eigen::Matrix4f Frame::getPoseInFrame(const FramePtr &frame) const
{
    return frame->toLocalCoordinateSystem(getGlobalPose());
}

Eigen::Vector3f Frame::getPositionInFrame(const FramePtr &frame) const
{
    return frame->toLocalCoordinateSystemVec(getGlobalPosition());
}

} //namespace VirtualRobot

