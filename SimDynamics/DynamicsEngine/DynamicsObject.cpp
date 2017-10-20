#include "DynamicsObject.h"

#include <VirtualRobot/Model/Frame.h>

namespace SimDynamics
{

    DynamicsObject::DynamicsObject(VirtualRobot::ModelLinkPtr o)
    {
        THROW_VR_EXCEPTION_IF(!o, "NULL object");
        sceneObject = o;
        //engineMutexPtr.reset(new std::recursive_mutex()); // may be overwritten by another mutex!
    }

    DynamicsObject::~DynamicsObject()
    {
    }

    std::string DynamicsObject::getName() const
    {
        if (!sceneObject)
            return std::string();
        return sceneObject->getName();
    }

    VirtualRobot::ModelLink::Physics::SimulationType DynamicsObject::getSimType() const
    {
        return sceneObject->getSimulationType();
    }

    void DynamicsObject::setPose(const Eigen::Matrix4f& pose)
    {
        MutexLockPtr lock = getScopedLock();
<<<<<<< HEAD

        if (sceneObject->getSimulationType() == VirtualRobot::ModelLink::Physics::eStatic)
=======
        if (sceneObject->getSimulationType() == VirtualRobot::SceneObject::Physics::eDynamic)
        {
            // moving dynamic objects is not allowed
            return;
        }
        if (sceneObject->getSimulationType() == VirtualRobot::SceneObject::Physics::eStatic)
>>>>>>> origin/master
        {
            VR_ERROR << "Could not move static object, use kinematic instead, aborting..." << endl;
            return;
        }
        try
        {
            sceneObject->setGlobalPose(pose);
        }
        catch(...) // robot node does not allow to set the pose
        {

        }
    }

    void DynamicsObject::setPosition(const Eigen::Vector3f& posMM)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Matrix4f pose = sceneObject->getGlobalPose();
        pose.block(0, 3, 3, 1) = posMM;
        setPose(pose);
    }

    VirtualRobot::ModelLinkPtr DynamicsObject::getSceneObject()
    {
        return sceneObject;
    }

    Eigen::Vector3f DynamicsObject::getLinearVelocity()
    {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f DynamicsObject::getAngularVelocity()
    {
        return Eigen::Vector3f::Zero();
    }

    void DynamicsObject::setLinearVelocity(const Eigen::Vector3f& vel)
    {

    }

    void DynamicsObject::setAngularVelocity(const Eigen::Vector3f& vel)
    {

    }

    void DynamicsObject::applyForce(const Eigen::Vector3f& force)
    {

    }

    void DynamicsObject::applyTorque(const Eigen::Vector3f& torque)
    {

    }

    void DynamicsObject::setMutex(std::shared_ptr<std::recursive_mutex> engineMutexPtr)
    {
        this->engineMutexPtr = engineMutexPtr;
    }

    void DynamicsObject::setSimType(VirtualRobot::ModelLink::Physics::SimulationType s)
    {
        sceneObject->setSimulationType(s);
    }

<<<<<<< HEAD
    void DynamicsObject::updateVisualization()
    {
        if (sceneObject)
            sceneObject->updatePose(true, true);
    }

=======
    void DynamicsObject::activate()
    {

    }
>>>>>>> origin/master

    DynamicsObject::MutexLockPtr DynamicsObject::getScopedLock()
    {
		std::shared_ptr< std::unique_lock<std::recursive_mutex> > scoped_lock;

		if (engineMutexPtr)
		{
			scoped_lock.reset(new std::unique_lock<std::recursive_mutex>(*engineMutexPtr));
		}

        return scoped_lock;
    }

} // namespace SimDynamics
