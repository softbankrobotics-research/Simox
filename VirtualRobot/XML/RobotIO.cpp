
#include "RobotIO.h"
#include "../RobotFactory.h"
#include "../Model/ModelNodeSet.h"
#include "../VirtualRobotException.h"
#include "../EndEffector/EndEffector.h"
#include "../EndEffector/EndEffectorActor.h"
#include "../Model/Nodes/ModelJointFixed.h"
#include "../Model/Nodes/ModelJointPrismatic.h"
#include "../Model/Nodes/ModelJointRevolute.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/VisualizationNode.h"
#include "../Visualization/TriMeshModel.h"
#include "../Model/ModelConfig.h"
#include "../Tools/RuntimeEnvironment.h"
#include "../CollisionDetection/CollisionModel.h"
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
	




    /**
     * This method parses the EndEffector which are child tags of the Robot tag.
     * Each EndEffector has a name, a base node, a list of static robot nodes and a list of actors.
     * Each actor itself consists of a list of robot nodes.
     *
     * The static parts and the actors are retrieved by delegating the processing
     * to RobotIO::processEndeffectorActorNode and RobotIO::processEndeffectorStaticNode.
     *
     * \return instance of VirtualRobot::EndEffector
     */
    EndEffectorPtr RobotIO::processEndeffectorNode(rapidxml::xml_node<char>* endeffectorXMLNode, RobotPtr robo)
    {
        std::string endeffectorName("");
        std::string baseNodeName;
        std::string tcpNodeName;
        std::string gcpNodeName;
        RobotNodePtr baseNode;
        RobotNodePtr tcpNode;
        RobotNodePtr gcpNode;

        // process attributes first
        rapidxml::xml_attribute<>* attr = endeffectorXMLNode->first_attribute();

        while (attr)
        {
            std::string attributeName = getLowerCase(attr->name());

            if ("name" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(!endeffectorName.empty(), "Endeffector tag has more than one <name> tag. Value of the first one is: " + endeffectorName);
                endeffectorName = attr->value();
                THROW_VR_EXCEPTION_IF(endeffectorName.empty(), "Endeffector tag does not specify a <name> tag.");
            }
            else if ("base" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(!baseNodeName.empty(), "Endeffector tag has more than one <base> tag. Value of the first one is: " + baseNodeName);
                baseNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(baseNodeName.empty(), "Endeffector tag does not specify a <base> tag.");
                baseNode = robo->getModelNode(baseNodeName);
                THROW_VR_EXCEPTION_IF(!baseNode, "base associated with <Endeffector> not available in the robot model.");
            }
            else if ("tcp" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(!tcpNodeName.empty(), "Endeffector tag has more than one <tcp> tag. Value of the first one is: " + tcpNodeName);
                tcpNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(tcpNodeName.empty(), "Endeffector tag does not specify a <tcp> tag.");
                tcpNode = robo->getModelNode(tcpNodeName);
                THROW_VR_EXCEPTION_IF(!tcpNode, "tcp associated with <Endeffector> not available in the robot model.");
            }
            else if ("gcp" == attributeName)
            {
                THROW_VR_EXCEPTION_IF(!gcpNodeName.empty(), "Endeffector tag has more than one <gcp> tag. Value of the first one is: " + gcpNodeName);
                gcpNodeName = attr->value();
                THROW_VR_EXCEPTION_IF(gcpNodeName.empty(), "Endeffector tag does not specify a <gcp> tag.");
                gcpNode = robo->getModelNode(gcpNodeName);
                THROW_VR_EXCEPTION_IF(!gcpNode, "gcp associated with <Endeffector> not available in the robot model.");
            }
            else
            {
                VR_WARNING << "Ignoring unknown attribute in EEF <" << endeffectorName << "> definition:" << attributeName << endl;
            }

            attr = attr->next_attribute();
        }

        std::vector<ModelLinkPtr> staticParts;
        std::vector<EndEffectorActorPtr> actors;
        std::vector< std::vector< RobotConfig::Configuration > > configDefinitions;
        std::vector< std::string > configNames;
        rapidxml::xml_node<>* node = endeffectorXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if ("actor" == nodeName)
            {
                actors.push_back(processEndeffectorActorNode(node, robo));
            }
            else if ("static" == nodeName)
            {
                if (staticParts.empty())
                {
                    processEndeffectorStaticNode(node, robo, staticParts);
                }
                else
                {
                    VR_ERROR << "There should only be one <static> tag inside <endeffector> tags" << endl;
                }
            }
            else if ("preshape" == nodeName)
            {
                bool cOK = processConfigurationNodeList(node, configDefinitions, configNames);
                THROW_VR_EXCEPTION_IF(!cOK, "Invalid Preshape defined in robot's eef tag '" << nodeName << "'." << endl);
            }
            else
            {
                THROW_VR_EXCEPTION("XML tag <" << nodeName << "> not supported in endeffector <" << nodeName << ">");
            }

            node = node->next_sibling();
        }

        if (!tcpNode)
        {
            tcpNode = baseNode;
        }

        if (!gcpNode)
        {
            gcpNode = tcpNode;
        }

        EndEffectorPtr endEffector(new EndEffector(endeffectorName, actors, staticParts, baseNode, tcpNode, gcpNode));

        // create & register configs
        THROW_VR_EXCEPTION_IF(configDefinitions.size() != configNames.size(), "Invalid Preshape definitions " << endl);

        for (size_t i = 0; i < configDefinitions.size(); i++)
        {
            RobotConfigPtr rc(new RobotConfig(robo, configNames[i], configDefinitions[i]));
            endEffector->registerPreshape(rc);
        }

        return endEffector;
    }


    /**
     * This method processes the attributes and the children of an actor tag which
     * itself is a child of the endeffector tag.
     * First the name attribute is retrieved and afterwards the child nodes are
     * processed which make up the actor.
     */
    EndEffectorActorPtr RobotIO::processEndeffectorActorNode(rapidxml::xml_node<char>* endeffectorActorXMLNode, RobotPtr robo)
    {
        std::string actorName = processNameAttribute(endeffectorActorXMLNode);
        THROW_VR_EXCEPTION_IF(actorName.empty(), "<Actor> tag inside <Endeffector> does not specify a <name> attribute.");
        std::vector<EndEffectorActor::ActorDefinition> actors;
        processActorNodeList(endeffectorActorXMLNode, robo, actors);

        EndEffectorActorPtr actor(new EndEffectorActor(actorName, actors));
        return actor;
    }


    /**
     * This method processes the children of the static tag which
     * itself is a child of the endeffector tag.
     */
	void RobotIO::processEndeffectorStaticNode(rapidxml::xml_node<char>* endeffectorStaticXMLNode, RobotPtr robo, std::vector<ModelLinkPtr>& staticNodesList)
    {
		std::vector<ModelNodePtr> tmpList;
        processNodeList(endeffectorStaticXMLNode, robo, tmpList, false);
		for (auto l : tmpList)
		{
			ModelLinkPtr m = std::dynamic_pointer_cast<ModelLink>(l);
			if (m)
				staticNodesList.push_back(m);
		}
    }


    /**
     * This method processes the \p parentNode Tag and extracts a list of \<Node name="xyz" speed="0123" /\> tags.
     * All other child tags raise a VirtualRobot::VirtualRobotException.
     * The resulting nodes are stored in \p nodeList.
     *
     * If the parameter \p clearList is true all elements from \p nodeList are removed.
     */
    void RobotIO::processActorNodeList(rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<EndEffectorActor::ActorDefinition>& actorList, bool clearList /*= true*/)
    {
        if (clearList)
        {
            actorList.clear();
        }

        std::string parentName = processNameAttribute(parentNode, true);
        std::string speedname("direction");
        rapidxml::xml_node<>* node = parentNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "node")
            {
                EndEffectorActor::ActorDefinition actor;
                std::string nodeNameAttr = processNameAttribute(node, true);
                THROW_VR_EXCEPTION_IF(nodeNameAttr.empty(), "Missing name attribute for <Node> belonging to Robot node set " << parentName);
                actor.robotNode = robot->getModelNode(nodeNameAttr);
                THROW_VR_EXCEPTION_IF(!actor.robotNode, "<node> tag with name '" << nodeNameAttr << "' not present in the current robot");
                actor.directionAndSpeed = processFloatAttribute(speedname, node, true);

                if (actor.directionAndSpeed == 0.0f)
                {
                    actor.directionAndSpeed = 1.0f;
                }

                actor.colMode = processEEFColAttributes(node, true);
                actorList.push_back(actor);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in <Actor> with name " << parentName);
            }

            node = node->next_sibling();
        }
    }


    EndEffectorActor::CollisionMode RobotIO::processEEFColAttributes(rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
    {
        THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute of NULL node" << endl);

        EndEffectorActor::CollisionMode result = EndEffectorActor::eNone;
        bool specified = false;
        std::string nodeNameAttr("");
        rapidxml::xml_attribute<char>* attr = node->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if ("considercollisions" == name)
            {
                std::string opt = getLowerCase(attr->value());
                specified = true;

                if (opt == "actors")
                {
                    result = EndEffectorActor::CollisionMode(result | EndEffectorActor::eActors);
                }
                else if (opt == "static")
                {
                    result = EndEffectorActor::CollisionMode(result | EndEffectorActor::eStatic);
                }
                else if (opt == "all")
                {
                    result = EndEffectorActor::eAll;
                }
                else if (opt == "none")
                {
                    result = EndEffectorActor::eNone;
                }
                else
                {
                    THROW_VR_EXCEPTION("<" << node->name() << "> considerCollisions attribute is unknowne: " << name);

                }
            }
            else
            {
                if (!allowOtherAttributes)
                {
                    THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
                }
            }

            attr = attr->next_attribute();
        }

        // standard behavior: check collisions with all actors and static part of EEF
        if (!specified)
        {
            result = EndEffectorActor::eAll;
        }

        return result;
    }



    RobotPtr RobotIO::loadRobotModel(const std::string &xmlFile, RobotIO::RobotDescription loadMode)
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

    VirtualRobot::RobotPtr RobotIO::createRobotModelFromString(const std::string& xmlString, const std::string& basePath, RobotDescription loadMode)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::RobotPtr robot;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* robotXMLNode = doc.first_node("robotmodel", 0, false);

            //robot = processRobot(robotXMLNode, basePath, loadMode);

            // todo...
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


    bool RobotIO::createEndEffectorsFromString(const RobotPtr &robot, const std::string &xmlString)
    {
        if (!robot)
            return false;

        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags

            rapidxml::xml_node<char>* eefNode = doc.first_node("endeffector", 0, false);
            while (eefNode)
            {
                EndEffectorPtr eef = processEndeffectorNode(eefNode, robot);
                if (eef)
                    robot->registerEndEffector(eef);
                else
                {
                    VR_ERROR << "Failed to process eef definition..." << endl;
                }
                eefNode = eefNode->next_sibling("endeffector", 0, false);
            }
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse eef in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return false;
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
            return false;
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing eef xml definition" << endl);
            return false;
        }

        delete[] y;
        return true;
    }

    bool RobotIO::loadEndEffectors(const RobotPtr &robot, const std::string &filename)
    {
        std::string fullFile = filename;

        if (!RuntimeEnvironment::getDataFileAbsolute(fullFile))
        {
            VR_ERROR << "Could not open XML file:" << filename << endl;
            return false;
        }

        // load file
        std::ifstream in(fullFile.c_str());

        if (!in.is_open())
        {
            VR_ERROR << "Could not open XML file:" << fullFile << endl;
            return false;
        }

        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string eefXML(buffer.str());
        in.close();

        return createEndEffectorsFromString(robot, eefXML);
    }

} // namespace VirtualRobot
