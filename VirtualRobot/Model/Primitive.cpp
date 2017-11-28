#include "Primitive.h"
#include "../Tools/MathTools.h"
#include "../XML/BaseIO.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

#include <boost/format.hpp>

namespace VirtualRobot
{
    namespace Primitive
    {

        std::string Primitive::getTransformString(int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<Transform>\n";
            result << BaseIO::getTransformXMLString(transform, tabs + 2);
            result << pre << "\t</Transform>\n";
            return result.str();
        }

        std::string Primitive::getXMLString(const std::string& type, const std::string& params, int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<" << type << " " << params << ">\n";
            result << getTransformString(tabs + 1);
            result << pre << "\t</" << type << ">\n";
            return result.str();
        }

        //derivate functions
        std::string Box::toXMLString(int tabs)
        {
            return getXMLString(
                       "Box",
                       (boost::format("width=\"%f\" height=\"%f\" depth=\"%f\"") % width % height % depth).str(),
                       tabs);
        }

        VisualizationPtr Box::getVisualization() const
        {
            auto visu = VisualizationFactory::getGlobalVisualizationFactory()->createBox(width, height, depth);
            visu->setGlobalPose(transform);
            return visu;
        }

        std::string Sphere::toXMLString(int tabs)
        {
            return getXMLString(
                        "Sphere",
                        (boost::format("radius=\"%f\"") % radius).str(),
                        tabs);
        }

        VisualizationPtr Sphere::getVisualization() const
        {
            auto visu = VisualizationFactory::getGlobalVisualizationFactory()->createSphere(radius);
            visu->setGlobalPose(transform);
            return visu;
        }

        std::string Cylinder::toXMLString(int tabs)
        {
            return getXMLString(
                       "Cylinder",
                       (boost::format("radius=\"%f\" height=\"%f\"") % radius % height).str(),
                       tabs);
        }

        VisualizationPtr Cylinder::getVisualization() const
        {
            auto visu = VisualizationFactory::getGlobalVisualizationFactory()->createCylinder(radius, height);
            visu->setGlobalPose(transform);
            return visu;
        }

    } //namespace Primitive
} //namespace VirtualRobot
