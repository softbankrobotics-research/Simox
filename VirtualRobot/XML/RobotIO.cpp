
#include "RobotIO.h"
#include "../RobotFactory.h"
#include "../Model/ModelNodeSet.h"
#include "../VirtualRobotException.h"
#include "../Model/Nodes/ModelJointFixed.h"
#include "../Model/Nodes/ModelJointPrismatic.h"
#include "../Model/Nodes/ModelJointRevolute.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/VisualizationNode.h"
#include "../Visualization/TriMeshModel.h"
#include "../Model/ModelConfig.h"
#include "../Tools/RuntimeEnvironment.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../Import/RobotImporterFactory.h"
#include "../Import/SimoxXMLFactory.h"
#include "FileIO.h"
#include "EndEffectorIO.h"
#include "rapidxml.hpp"


#include <vector>
#include <fstream>
#include <iostream>

namespace VirtualRobot
{



    RobotIO::RobotIO()
    {
    }

    RobotIO::~RobotIO()
    {
    }

    bool RobotIO::searchFile(std::string &filename, const std::string &basePath)
    {
        // check for relative path
        std::string filenameLocal = basePath + FileIO::getPathSeparator() + filename;
        if (RuntimeEnvironment::getDataFileAbsolute(filenameLocal))
            filename = filenameLocal;
        else
        {
            // check for global path
            if (!RuntimeEnvironment::getDataFileAbsolute(filename))
                return false;
        }
        return true;
    }



	/*
    bool RobotIO::processSensor(RobotNodePtr rn, rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath)
    {
        if (!rn || !sensorXMLNode)
        {
            VR_ERROR << "NULL DATA ?!" << endl;
            return false;
        }

        rapidxml::xml_attribute<>* attr;
        std::string sensorType;

        attr = sensorXMLNode->first_attribute("type", 0, false);

        if (attr)
        {
            sensorType = getLowerCase(attr->value());
        }
        else
        {
            VR_WARNING << "No 'type' attribute for <Sensor> tag. Skipping Sensor definition of RobotNode " << rn->getName() << "!" << endl;
            return false;
        }

        SensorPtr s;


        SensorFactoryPtr sensorFactory = SensorFactory::fromName(sensorType, NULL);

        if (sensorFactory)
        {
            s = sensorFactory->createSensor(rn, sensorXMLNode, loadMode, basePath);
        }
        else
        {
            VR_WARNING << "No Factory found for sensor of type " << sensorType << ". Skipping Sensor definition of RobotNode " << rn->getName() << "!" << endl;
            return false;
        }

        return rn->registerSensor(s);
    }*/


    ModelPtr RobotIO::loadRobotModel(const std::string &xmlFile, RobotIO::RobotDescription loadMode)
    {
        std::string fullFile = xmlFile;

        if (!RuntimeEnvironment::getDataFileAbsolute(fullFile))
        {
            VR_ERROR << "Could not open XML file:" << xmlFile << endl;
            return RobotPtr();
        }

        // load file
        std::ifstream in(fullFile.c_str());

        if (!in.is_open())
        {
            VR_ERROR << "Could not open XML file:" << fullFile << endl;
            return RobotPtr();
        }

        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string robotXML(buffer.str());
        boost::filesystem::path filenameBaseComplete(xmlFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();

        in.close();

        VirtualRobot::RobotPtr res = createRobotModelFromString(robotXML, basePath, loadMode);

        if (!res)
        {
            VR_ERROR << "Error while parsing file " << fullFile << endl;
        }

        res->applyJointValues();
        res->setFilename(xmlFile);
        return res;
    }

    ModelPtr RobotIO::createRobotModelFromString(const std::string& xmlString, const std::string& basePath, RobotDescription loadMode)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::RobotPtr robot;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* robotXMLNode = doc.first_node("modeldescription", 0, false);

            robot = processModelDescription(robotXMLNode, basePath, loadMode);
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return RobotPtr();
        }
        catch (VirtualRobot::VirtualRobotException&)
        {
            delete[] y;
            // rethrow the current exception
            throw;
        }
        catch (std::exception& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                               << "Error code:" << e.what() << endl);
            return RobotPtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return RobotPtr();
        }

        delete[] y;

        if (loadMode == RobotIO::eCollisionModel)
        {
            // use collision visualization to build main visualization
            // todo
            //robot->createVisualizationFromCollisionModels();
        }

        return robot;
    }

    ModelPtr RobotIO::processModelDescription(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, RobotIO::RobotDescription loadMode)
    {
        THROW_VR_EXCEPTION_IF(!robotXMLNode, "No <ModelDescription> tag in XML definition");
        ModelPtr robot;

        // check URDF
        rapidxml::xml_node<>* XMLNode = robotXMLNode->first_node("urdf", 0, false);
        if (XMLNode)
        {
            std::string filename = XMLNode->value();
            bool fileOK = searchFile(filename, basePath);
            THROW_VR_EXCEPTION_IF(!fileOK, "Could not find file " << filename);

            RobotImporterFactoryPtr rf = RobotImporterFactory::fromName("SimoxURDF", NULL);
            if (!rf)
            {
                THROW_VR_EXCEPTION("Could not instanciate URDF loader, most likely Simox was not compiled with URDF support...");
            }
            robot = rf->loadFromFile(filename, loadMode);

            XMLNode = XMLNode->next_sibling("urdf", 0, false);
            THROW_VR_EXCEPTION_IF (XMLNode, "Two urdf tags not allowed in a model description");
        }

        // check SimoxXML
        XMLNode = robotXMLNode->first_node("simoxxml", 0, false);
        if (XMLNode)
        {
            THROW_VR_EXCEPTION_IF(robot, "Do not mix URDF and SimoxXML tags...");
            std::string filename = XMLNode->value();

            bool fileOK = searchFile(filename, basePath);
            THROW_VR_EXCEPTION_IF(!fileOK, "Could not find file " << filename);

            RobotImporterFactoryPtr rf = RobotImporterFactory::fromName("SimoxXML", NULL);
            THROW_VR_EXCEPTION_IF(!rf, "Could not instanciate SimoxXML loader...");
            robot = rf->loadFromFile(filename, loadMode);

            XMLNode = XMLNode->next_sibling("simoxxml", 0, false);
            THROW_VR_EXCEPTION_IF (XMLNode, "Two SimoxXML tags not allowed in a model description");
        }

        // load EndEffectors
        XMLNode = robotXMLNode->first_node("endeffector", 0, false);
        while (XMLNode)
        {
            THROW_VR_EXCEPTION_IF(!robot, "Could not process end effectors due to missing model defintion...");

            std::string filename = XMLNode->value();

            bool fileOK = searchFile(filename, basePath);
            THROW_VR_EXCEPTION_IF(!fileOK, "Could not find file " << filename);

            bool eefOK = EndEffectorIO::loadEndEffectors(robot, filename);
            THROW_VR_EXCEPTION_IF(!eefOK, "Could not parse end effector defintion...");

            XMLNode = XMLNode->next_sibling("endeffector", 0, false);
        }


        // load Joint/Link/modelNodeSets
        // todo....

        return robot;
    }


} // namespace VirtualRobot
