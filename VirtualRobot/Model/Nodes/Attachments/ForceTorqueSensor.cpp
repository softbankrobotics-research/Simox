
#include "ForceTorqueSensor.h"

namespace VirtualRobot
{
    ForceTorqueSensor::ForceTorqueSensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Sensor(name, localTransformation, visualizationType)
        , forceTorqueValues(6)
    {
        forceTorqueValues.setZero();
        initVisualization();
    }


    ForceTorqueSensor::~ForceTorqueSensor()
    {
    }


    bool ForceTorqueSensor::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ForceTorqueSensor::getType()
    {
        return "forcetorque";
    }

    ModelNodeAttachmentPtr ForceTorqueSensor::clone()
    {
        ForceTorqueSensorPtr result(new ForceTorqueSensor(name, localTransformation, visualizationType));
        result->updateSensors(forceTorqueValues);
        return result;
    }

    void ForceTorqueSensor::initVisualization()
    {
        VisualizationFactoryPtr factory;
        if (visualizationType.empty())
        {
            factory = VirtualRobot::VisualizationFactory::first(NULL);
        } else
        {
            factory = VirtualRobot::VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }

    Eigen::Vector3f ForceTorqueSensor::getForce() const
    {
        return forceTorqueValues.head(3);
    }

    Eigen::Vector3f ForceTorqueSensor::getTorque() const
    {
        return forceTorqueValues.tail(3);
    }

    const Eigen::VectorXf& ForceTorqueSensor::getForceTorque()
    {
        return forceTorqueValues;
    }

    Eigen::Vector3f ForceTorqueSensor::getAxisTorque()
    {
        Eigen::Vector3f torqueVector = forceTorqueValues.tail(3);

        // project onto joint axis
        //RobotNodePtr rn(robotNode);
        Eigen::Vector3f zAxis = this->globalPose.block(0, 2, 3, 1);
        Eigen::Vector3f axisTorque = (torqueVector.dot(zAxis)) * zAxis;

        return axisTorque;
    }

    void ForceTorqueSensor::updateSensors(const Eigen::VectorXf& newForceTorque)
    {
        forceTorqueValues = newForceTorque;
    }

}

