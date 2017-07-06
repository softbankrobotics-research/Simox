#include "Orientation.h"

namespace VirtualRobot
{

Orientation::Orientation() : rot(Eigen::Matrix3f::Identity())
{

}

Orientation::~Orientation()
{

}

Orientation::Orientation(const Eigen::Matrix3f &rotationMatrix) : rot(rotationMatrix)
{

}

Orientation::Orientation(const MathTools::Quaternion &q)
{
    rot = MathTools::quat2eigen4f(q).block(0,0,3,3);
}

Orientation::Orientation(const Eigen::Vector3f &rpy)
{
    rot = MathTools::rpy2eigen4f(rpy).block(0,0,3,3);
}

Eigen::Vector3f Orientation::rpy() const
{
    return MathTools::eigen3f2rpy(rot);
}

Eigen::Matrix3f Orientation::matrix() const
{
    return rot;
}

MathTools::Quaternion Orientation::quaternion() const
{
    return MathTools::eigen3f2quat(rot);
}


}
