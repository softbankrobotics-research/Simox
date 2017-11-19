
#include "ForceTorqueSensor.h"
#include "../../XML/BaseIO.h"

namespace VirtualRobot
{
    ForceTorqueSensor::ForceTorqueSensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Sensor(name, localTransformation, visualizationType)
        , forceTorqueValues(6)
    {
        forceTorqueValues.setZero();
    }


    ForceTorqueSensor::~ForceTorqueSensor()
    {
    }


    bool ForceTorqueSensor::isAttachable(const ModelNodePtr &node)
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

    std::string ForceTorqueSensor::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }
        std::stringstream ss;
        ss << t << "<Sensor type='forcetorque' name='" << name <<"'>\n";
        std::string pre2 = t + "\t";
        std::string pre3 = pre2 + "\t";
        ss << pre2 << "<Transform>" << endl;
        ss << BaseIO::toXML(localTransformation, pre3);
        ss << pre2 << "</Transform>" << endl;
        ss << t << "</Sensor>\n";

        return ss.str();
    }

}

