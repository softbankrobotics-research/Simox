
#include "PositionSensor.h"
#include "../../XML/BaseIO.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    PositionSensor::PositionSensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Sensor(name, localTransformation, visualizationType)
    {
        initVisualization();
    }


    PositionSensor::~PositionSensor()
    {
    }


    bool PositionSensor::isAttachable(const ModelNodePtr &node)
    {
        return true;
    }

    std::string PositionSensor::getType()
    {
        return "position";
    }

    ModelNodeAttachmentPtr PositionSensor::clone()
    {
        ModelNodeAttachmentPtr result(new PositionSensor(name, localTransformation, visualizationType));
        return result;
    }

    void PositionSensor::initVisualization()
    {
        VisualizationFactoryPtr factory = VisualizationFactory::getGlobalVisualizationFactory();

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }


    std::string PositionSensor::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }
        std::stringstream ss;
        ss << t << "<Sensor type='position' name='" << name <<"'>\n";
        std::string pre2 = t + "\t";
        std::string pre3 = pre2 + "\t";
        ss << pre2 << "<Transform>" << endl;
        ss << BaseIO::toXML(localTransformation, pre3);
        ss << pre2 << "</Transform>" << endl;
        ss << t << "</Sensor>\n";

        return ss.str();
    }

}

