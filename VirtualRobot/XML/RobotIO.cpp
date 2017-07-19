
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


    std::map<std::string, int> RobotIO::robot_name_counter;

    RobotIO::RobotIO()
    {
    }

    RobotIO::~RobotIO()
    {
    }



    void RobotIO::processChildNode(rapidxml::xml_node<char>* childXMLNode, std::vector<std::string>& childrenNames)
    {
        THROW_VR_EXCEPTION_IF(!childXMLNode, "NULL data for childXMLNode in processChildNode()")

        rapidxml::xml_attribute<>* attr;
        attr = childXMLNode->first_attribute("name", 0, false);
        THROW_VR_EXCEPTION_IF(!attr, "Expecting 'name' attribute in <Child> tag..." << endl)

        std::string s(attr->value());
        childrenNames.push_back(s);
    }

    void RobotIO::processChildFromRobotNode(rapidxml::xml_node<char>* childXMLNode, const std::string& nodeName, std::vector< ChildFromRobotDef >& childrenFromRobot)
    {
        rapidxml::xml_attribute<>* attr;

        THROW_VR_EXCEPTION_IF(!childXMLNode, "NULL data for childXMLNode in processChildFromRobotNode()")

        ChildFromRobotDef d;

        rapidxml::xml_node<>* fileXMLNode = childXMLNode->first_node("file", 0, false);
        int counter = 0;

        while (fileXMLNode)
        {
            d.filename = fileXMLNode->value();

            d.importEEF = true;
            attr = fileXMLNode->first_attribute("importEEF", 0, false);

            if (attr)
            {
                if (!isTrue(attr->value()))
                {
                    d.importEEF = false;
                }
            }

            childrenFromRobot.push_back(d);
            fileXMLNode = fileXMLNode->next_sibling("file", 0, false);
        }

        THROW_VR_EXCEPTION_IF(((!counter) == 0), "Missing file for <ChildFromRobot> tag (in node '" << nodeName << "')." << endl);
    }


    /**
     * This method processes Limits tags.
     * The values for the attributes "lo" and "hi" are extracted based on the
     * "unit" or "units" attribute.
     *
     * The values are stored in \p jointLimitLo and \p jointLimitHi
     */
    void RobotIO::processLimitsNode(rapidxml::xml_node<char>* limitsXMLNode, float& jointLimitLo, float& jointLimitHi)
    {
        THROW_VR_EXCEPTION_IF(!limitsXMLNode, "NULL data for limitsXMLNode in processLimitsNode()");

        Units unit = getUnitsAttribute(limitsXMLNode, Units::eIgnore);

        try
        {
            jointLimitLo = getFloatByAttributeName(limitsXMLNode, "lo");
        }
        catch (...)
        {
            if (unit.isLength())
            {
                VR_WARNING << "No 'lo' attribute in <Limits> tag. Assuming -1000 [mm]." << endl;
                jointLimitLo = -1000.0f;
                unit = Units("mm");
            }
            else
            {
                VR_WARNING << "No 'lo' attribute in <Limits> tag. Assuming -180 [deg]." << endl;
                jointLimitLo = (float)(-M_PI);
                unit = Units("rad");
            }
        }

        try
        {
            jointLimitHi = getFloatByAttributeName(limitsXMLNode, "hi");
        }
        catch (...)
        {
            if (unit.isLength())
            {
                VR_WARNING << "No 'hi' attribute in <Limits> tag. Assuming 1000 [mm]." << endl;
                jointLimitLo = 1000.0f;
                unit = Units("mm");
            }
            else
            {
                VR_WARNING << "No 'hi' attribute in <Limits> tag. Assuming 180 [deg]." << endl;
                jointLimitHi = (float)M_PI;
                unit = Units("rad");
            }
        }

        // if values are stored as degrees convert them to radian
        if (unit.isAngle())
        {
            jointLimitLo = unit.toRadian(jointLimitLo);
            jointLimitHi = unit.toRadian(jointLimitHi);
        }

        if (unit.isLength())
        {
            jointLimitLo = unit.toMillimeter(jointLimitLo);
            jointLimitHi = unit.toMillimeter(jointLimitHi);
        }
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
	

	ModelLinkPtr RobotIO::processLinkNode(const std::string& robotNodeName,
		RobotPtr robot,
		VisualizationNodePtr visualizationNode,
		CollisionModelPtr collisionModel,
		ModelLink::Physics& physics, 
		const Eigen::Matrix4f& transformationMatrix
		)
	{
		Eigen::Matrix4f preJointTransform = transformationMatrix;//Eigen::Matrix4f::Identity();
		ModelLinkPtr robotNode(new ModelLink(robot, robotNodeName, transformationMatrix, visualizationNode, collisionModel, physics));
		return robotNode;
	}

    ModelJointPtr RobotIO::processJointNode(rapidxml::xml_node<char>* jointXMLNode, 
											const std::string& robotNodeName,
											ModelPtr robot,
											const Eigen::Matrix4f& transformationMatrix
                                          )
    {
        float jointLimitLow = (float) - M_PI;
        float jointLimitHigh = (float)M_PI;

        Eigen::Matrix4f preJointTransform = transformationMatrix;//Eigen::Matrix4f::Identity();
        Eigen::Vector3f axis = Eigen::Vector3f::Zero();
        Eigen::Vector3f translationDir = Eigen::Vector3f::Zero();

        std::vector< std::string > propagateJVName;
        std::vector< float > propagateJVFactor;

        rapidxml::xml_attribute<>* attr;
        float jointOffset = 0.0f;
        float initialvalue = 0.0f;
        std::string jointType;

		ModelJointPtr robotNode;

        if (!jointXMLNode)
        {
            // no <Joint> tag -> fixed joint
            robotNode.reset(new ModelJointFixed(robot, robotNodeName, preJointTransform));
            return robotNode;
        }

        attr = jointXMLNode->first_attribute("type", 0, false);

        if (attr)
        {
            jointType = getLowerCase(attr->value());
        }
        else
        {
            VR_WARNING << "No 'type' attribute for <Joint> tag. Assuming fixed joint for RobotNode " << robotNodeName << "!" << endl;
			robotNode.reset(new ModelJointFixed(robot, robotNodeName, preJointTransform));
			return robotNode;
        }

        attr = jointXMLNode->first_attribute("offset", 0, false);

        if (attr)
        {
            jointOffset = convertToFloat(attr->value());
        }

        attr = jointXMLNode->first_attribute("initialvalue", 0, false);

        if (attr)
        {
            initialvalue = convertToFloat(attr->value());
        }

        rapidxml::xml_node<>* node = jointXMLNode->first_node();
        rapidxml::xml_node<>* tmpXMLNodeAxis = NULL;
        rapidxml::xml_node<>* tmpXMLNodeTranslation = NULL;
        rapidxml::xml_node<>* limitsNode = NULL;

        float maxVelocity = -1.0f; // m/s
        float maxAcceleration = -1.0f; // m/s^2
        float maxTorque = -1.0f; // Nm

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "limits")
            {
                THROW_VR_EXCEPTION_IF(limitsNode, "Multiple limits definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                limitsNode = node;
                processLimitsNode(limitsNode, jointLimitLow, jointLimitHigh);
            }
            else if (nodeName == "axis")
            {
                THROW_VR_EXCEPTION_IF(tmpXMLNodeAxis, "Multiple axis definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                tmpXMLNodeAxis = node;
            }
            else if (nodeName == "translationdirection")
            {
                THROW_VR_EXCEPTION_IF(tmpXMLNodeTranslation, "Multiple translation definitions in <Joint> tag of robot node <" << robotNodeName << ">." << endl);
                tmpXMLNodeTranslation = node;
            }
            else if (nodeName == "maxvelocity")
            {
                maxVelocity = getFloatByAttributeName(node, "value");

                // convert to m/s
                std::vector< Units > unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (size_t i = 0; i < unitsAttr.size(); i++)
                {
                    if (unitsAttr[i].isTime())
                    {
                        uTime = unitsAttr[i];
                    }

                    if (unitsAttr[i].isLength())
                    {
                        uLength = unitsAttr[i];
                    }

                    if (unitsAttr[i].isAngle())
                    {
                        uAngle = unitsAttr[i];
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= 60.0f;
                }

                if (uTime.isHour())
                {
                    factor /= 3600.0f;
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0f);
                }

                maxVelocity *= factor;

            }
            else if (nodeName == "maxacceleration")
            {
                maxAcceleration = getFloatByAttributeName(node, "value");

                // convert to m/s^2
                std::vector< Units > unitsAttr = getUnitsAttributes(node);
                Units uTime("sec");
                Units uLength("m");
                Units uAngle("rad");

                for (size_t i = 0; i < unitsAttr.size(); i++)
                {
                    if (unitsAttr[i].isTime())
                    {
                        uTime = unitsAttr[i];
                    }

                    if (unitsAttr[i].isLength())
                    {
                        uLength = unitsAttr[i];
                    }

                    if (unitsAttr[i].isAngle())
                    {
                        uAngle = unitsAttr[i];
                    }
                }

                float factor = 1.0f;

                if (uTime.isMinute())
                {
                    factor /= 3600.0f;
                }

                if (uTime.isHour())
                {
                    factor /= 12960000.0f;
                }

                if (uLength.isMillimeter())
                {
                    factor *= 0.001f;
                }

                if (uAngle.isDegree())
                {
                    factor *= float(M_PI / 180.0f);
                }

                maxAcceleration *= factor;

            }
            else if (nodeName == "maxtorque")
            {
                maxTorque = getFloatByAttributeName(node, "value");
                // convert to Nm
                std::vector< Units > unitsAttr = getUnitsAttributes(node);
                Units uLength("m");

                for (size_t i = 0; i < unitsAttr.size(); i++)
                {
                    if (unitsAttr[i].isLength())
                    {
                        uLength = unitsAttr[i];
                    }
                }

                float factor = 1.0f;

                if (uLength.isMillimeter())
                {
                    factor *= 1000.0f;
                }

                maxTorque *= factor;
            }
            else if (nodeName == "propagatejointvalue")
            {
                rapidxml::xml_attribute<>* attrPropa;
                attrPropa = node->first_attribute("name", 0, false);
                THROW_VR_EXCEPTION_IF(!attrPropa, "Expecting 'name' attribute in <PropagateJointValue> tag..." << endl);

                std::string s(attrPropa->value());
                propagateJVName.push_back(s);
                float f = 1.0f;
                attrPropa = node->first_attribute("factor", 0, false);

                if (attrPropa)
                {
                    f = convertToFloat(attrPropa->value());
                }

                propagateJVFactor.push_back(f);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in <Joint> tag of RobotNode <" << robotNodeName << ">." << endl);
            }

            node = node->next_sibling();
        }

        if (jointType == "revolute")
        {
			THROW_VR_EXCEPTION_IF(tmpXMLNodeTranslation, "Translation tags not allowed in revolute joints");

            if (tmpXMLNodeAxis)
            {
                axis[0] = getFloatByAttributeName(tmpXMLNodeAxis, "x");
                axis[1] = getFloatByAttributeName(tmpXMLNodeAxis, "y");
                axis[2] = getFloatByAttributeName(tmpXMLNodeAxis, "z");
            }
            else
            {
                // silently setting axis to (0,0,1)
                //VR_WARNING << "Joint '" << robotNodeName << "' without 'axis' tag. Setting rotation axis to (0,0,1)." << endl;
                axis << 0, 0, 1.0f;
                //THROW_VR_EXCEPTION("joint '" << robotNodeName << "' wrongly defined, expecting 'axis' tag." << endl);
            }
			robotNode.reset(new ModelJointRevolute(robot, robotNodeName, preJointTransform, jointLimitLow, jointLimitHigh, axis, jointOffset));
        }
        else if (jointType == "prismatic")
        {
			THROW_VR_EXCEPTION_IF(tmpXMLNodeAxis, "Axis tags not allowed in primsatic joints");
			if (tmpXMLNodeTranslation)
            {
                translationDir[0] = getFloatByAttributeName(tmpXMLNodeTranslation, "x");
                translationDir[1] = getFloatByAttributeName(tmpXMLNodeTranslation, "y");
                translationDir[2] = getFloatByAttributeName(tmpXMLNodeTranslation, "z");
            }
            else
            {
                THROW_VR_EXCEPTION("Prismatic joint '" << robotNodeName << "' wrongly defined, expecting 'TranslationDirection' tag." << endl);
            }
			robotNode.reset(new ModelJointPrismatic(robot, robotNodeName, preJointTransform, jointLimitLow, jointLimitHigh, translationDir, jointOffset));
		}
		else
		{
			THROW_VR_EXCEPTION("Joint type not known:" << jointType << endl);
		}

        robotNode->setMaxVelocity(maxVelocity);
        robotNode->setMaxAcceleration(maxAcceleration);
        robotNode->setMaxTorque(maxTorque);

        robotNode->setJointValueNoUpdate(initialvalue);

        VR_ASSERT(propagateJVName.size() == propagateJVFactor.size());

        for (size_t i = 0; i < propagateJVName.size(); i++)
        {
            robotNode->propagateJointValue(propagateJVName[i], propagateJVFactor[i]);
        }
        return robotNode;
    }

    RobotNodePtr RobotIO::processRobotNode(rapidxml::xml_node<char>* robotNodeXMLNode,
                                           RobotPtr robo,
                                           const std::string& basePath,
                                           int& robotNodeCounter,
                                           std::vector< std::string >& childrenNames,
                                           std::vector< ChildFromRobotDef >& childrenFromRobot,
                                           RobotDescription loadMode)
    {
        childrenFromRobot.clear();
        THROW_VR_EXCEPTION_IF(!robotNodeXMLNode, "NULL data in processRobotNode");

        // get name
        std::string robotNodeName = processNameAttribute(robotNodeXMLNode);

        if (robotNodeName.empty())
        {
            std::stringstream ss;
            ss << robo->getType() << "_Node_" << robotNodeCounter;
            robotNodeName = ss.str();
            robotNodeCounter++;
            VR_WARNING << "RobotNode definition expects attribute 'name'. Setting name to " << robotNodeName << endl;
        }


        // visu data
        bool visuProcessed = false;
        //bool enableVisu = true;
        bool useAsColModel;

        // collision information
        bool colProcessed = false;

        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        RobotNodePtr robotNode;
        ModelLink::Physics physics;
        bool physicsDefined = false;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

        rapidxml::xml_node<>* node = robotNodeXMLNode->first_node();
        rapidxml::xml_node<>* jointNodeXML = NULL;

        std::vector< rapidxml::xml_node<>* > sensorTags;

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                if (loadMode == eFull)
                {
                    THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in RobotNode '" << robotNodeName << "'." << endl);
                    visualizationNode = processVisualizationTag(node, robotNodeName, basePath, useAsColModel);
                    visuProcessed = true;

                    if (useAsColModel)
                    {
                        THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in RobotNode '" << robotNodeName << "'." << endl);
                        std::string colModelName = robotNodeName;
                        colModelName += "_VISU_ColModel";
                        // clone model
                        VisualizationNodePtr visualizationNodeClone = visualizationNode->clone();
                        // todo: ID?
                        collisionModel.reset(new CollisionModel(visualizationNodeClone, colModelName, CollisionCheckerPtr()));
                        colProcessed = true;
                    }
                }
                else if (loadMode == eCollisionModel)
                {
                    VisualizationNodePtr visualizationNodeCM = checkUseAsColModel(node, robotNodeName, basePath);

                    if (visualizationNodeCM)
                    {
                        useAsColModel = true;
                        THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in RobotNode '" << robotNodeName << "'." << endl);
                        std::string colModelName = robotNodeName;
                        colModelName += "_VISU_ColModel";
                        // todo: ID?
                        collisionModel.reset(new CollisionModel(visualizationNodeCM, colModelName, CollisionCheckerPtr()));
                        colProcessed = true;
                    }
                }// else silently ignore tag
            }
            else if (nodeName == "collisionmodel")
            {
                if (loadMode == eFull || loadMode == eCollisionModel)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in RobotNode '" << robotNodeName << "'." << endl);
                    collisionModel = processCollisionTag(node, robotNodeName, basePath);
                    colProcessed = true;
                } // else silently ignore tag
            }
            else if (nodeName == "child")
            {
                processChildNode(node, childrenNames);
            }
            else if (nodeName == "childfromrobot")
            {
                processChildFromRobotNode(node, robotNodeName, childrenFromRobot);
            }
            else if (nodeName == "joint")
            {
                THROW_VR_EXCEPTION_IF(jointNodeXML, "Two joint tags defined in RobotNode '" << robotNodeName << "'." << endl);
                jointNodeXML = node;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined, "Two physics tags defined in RobotNode '" << robotNodeName << "'." << endl);
                processPhysicsTag(node, robotNodeName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "transform")
            {
                processTransformNode(robotNodeXMLNode, robotNodeName, transformMatrix);
            }
            else if (nodeName == "sensor")
            {
                sensorTags.push_back(node);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in RobotNode <" << robotNodeName << ">." << endl);
            }

            node = node->next_sibling();
        }

		THROW_VR_EXCEPTION_IF(jointNodeXML && (visualizationNode || collisionModel), "Visualization and/or collision models are not allowed to be defined in joint node:" + robotNodeName);
		THROW_VR_EXCEPTION_IF(jointNodeXML && (physicsDefined), "Physics properties are not allowed to be defined in joint node:" + robotNodeName);
		//create joint from xml data

		if (jointNodeXML)
			robotNode = processJointNode(jointNodeXML, robotNodeName, robo, transformMatrix);
		else
			robotNode = processLinkNode(robotNodeName, robo, visualizationNode, collisionModel, physics, transformMatrix);


        // process sensors
        /*for (size_t i = 0; i < sensorTags.size(); i++)
        {
            processSensor(robotNode, sensorTags[i], loadMode, basePath);
        }*/

        return robotNode;
    }


    VisualizationNodePtr RobotIO::checkUseAsColModel(rapidxml::xml_node<>* visuXMLNode, const std::string& /*robotNodeName*/, const std::string& basePath)
    {
        bool enableVisu = true;
        //bool coordAxis = false;
        //float coordAxisFactor = 1.0f;
        //std::string coordAxisText = "";
        //std::string visuCoordType = "";
        std::string visuFileType = "";
        std::string visuFile = "";
        rapidxml::xml_attribute<>* attr;
        VisualizationNodePtr visualizationNode;

        if (!visuXMLNode)
        {
            return visualizationNode;
        }

        attr = visuXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableVisu = isTrue(attr->value());
        }

        if (enableVisu)
        {
            rapidxml::xml_node<>* visuFileXMLNode = visuXMLNode->first_node("file", 0, false);

            if (visuFileXMLNode)
            {
                attr = visuFileXMLNode->first_attribute("type", 0, false);
                THROW_VR_EXCEPTION_IF(!attr, "Missing 'type' attribute in <Visualization> tag." << endl);
                visuFileType = attr->value();
                getLowerCase(visuFileType);
                visuFile = processFileNode(visuFileXMLNode, basePath);
            }

            rapidxml::xml_node<>* useColModel = visuXMLNode->first_node("useascollisionmodel", 0, false);

            if (useColModel && visuFile != "")
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(visuFileType, NULL);

                if (visualizationFactory)
                {
                    visualizationNode = visualizationFactory->getVisualizationFromFile(visuFile);
                }
                else
                {
                    VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data." << endl;
                }
            }
        }

        return visualizationNode;
    }

    RobotPtr RobotIO::processRobot(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, RobotDescription loadMode)
    {
        THROW_VR_EXCEPTION_IF(!robotXMLNode, "No <Robot> tag in XML definition");

        // process Attributes
        std::string robotRoot;
        RobotPtr robo;
        robo = processRobotAttributes(robotXMLNode, robotRoot);

        // process xml nodes
        std::map< RobotNodePtr, std::vector< ChildFromRobotDef > > childrenFromRobotFilesMap;
        std::vector<rapidxml::xml_node<char>* > robotNodeSetNodes;
        std::vector<rapidxml::xml_node<char>* > endeffectorNodes;

        processRobotChildNodes(robotXMLNode, robo, robotRoot, basePath, childrenFromRobotFilesMap, robotNodeSetNodes, endeffectorNodes, loadMode);

        // process childfromrobot tags
        std::map< RobotNodePtr, std::vector< ChildFromRobotDef > >::iterator iter = childrenFromRobotFilesMap.begin();

        while (iter != childrenFromRobotFilesMap.end())
        {
            std::vector< ChildFromRobotDef > childrenFromRobot = iter->second;
            RobotNodePtr node = iter->first;

            for (unsigned int i = 0; i < childrenFromRobot.size(); i++)
            {
                boost::filesystem::path filenameNew(childrenFromRobot[i].filename);
                boost::filesystem::path filenameBasePath(basePath);

                boost::filesystem::path filenameNewComplete = boost::filesystem::operator/(filenameBasePath, filenameNew);
                VR_INFO << "Searching robot: " << filenameNewComplete.string() << endl;

                try
                {
                    THROW_VR_EXCEPTION_IF(!boost::filesystem::exists(filenameNewComplete), "File <" << filenameNewComplete.string() << "> does not exist." << endl);
                }
                catch (...)
                {
                    THROW_VR_EXCEPTION("Error while processing file <" << filenameNewComplete.string() << ">." << endl);
                }

                RobotPtr r = loadRobot(filenameNewComplete.string(), loadMode);
                THROW_VR_EXCEPTION_IF(!r, "Could not add child-from-robot due to failed loading of robot from file" << childrenFromRobot[i].filename);
                RobotNodePtr root = r->getRootNode();
                THROW_VR_EXCEPTION_IF(!root, "Could not add child-from-robot. No root node in file" << childrenFromRobot[i].filename);

                RobotNodePtr rootNew = root->clone(robo, true, node);
                THROW_VR_EXCEPTION_IF(!rootNew, "Clone failed. Could not add child-from-robot from file " << childrenFromRobot[i].filename);

                std::vector<EndEffectorPtr> eefs = r->getEndEffectors();

                for (std::vector<EndEffectorPtr>::iterator eef = eefs.begin(); eef != eefs.end(); eef++)
                {
                    (*eef)->clone(robo);
                }

                std::vector<RobotNodeSetPtr> nodeSets = r->getModelNodeSets();

                for (std::vector<RobotNodeSetPtr>::iterator ns = nodeSets.begin(); ns != nodeSets.end(); ns++)
                {
                    (*ns)->clone(robo);
                }

                // already performed in root->clone
                //node->attachChild(rootNew);
            }

            iter++;
        }

        //std::vector<RobotNodeSetPtr> robotNodeSets
        for (unsigned int i = 0; i < endeffectorNodes.size(); ++i)
        {
            EndEffectorPtr eef = processEndeffectorNode(endeffectorNodes[i], robo);
            robo->registerEndEffector(eef);
        }

        int rnsCounter = 0;

        for (int i = 0; i < (int)robotNodeSetNodes.size(); ++i)
        {
            RobotNodeSetPtr rns = processRobotNodeSet(robotNodeSetNodes[i], robo, robotRoot, rnsCounter);
            //nodeSets.push_back(rns);
        }

        std::vector<RobotNodePtr> nodes;
        robo->getModelNodes(nodes);
        RobotNodePtr root = robo->getRootNode();

        for (size_t i = 0; i < nodes.size(); i++)
        {
            if (nodes[i] != root && !(nodes[i]->getParentNode()))
            {
                THROW_VR_EXCEPTION("Node without parent: " << nodes[i]->getName());
            }
        }

        return robo;
    }


    RobotPtr RobotIO::processRobotAttributes(rapidxml::xml_node<char>* robotXMLNode, std::string& robotRoot)
    {
        std::string robotName;
        std::string robotType;

        // process attributes of robot
        rapidxml::xml_attribute<>* attr;
        attr = robotXMLNode->first_attribute("type", 0, false);

        if (!attr)
        {
            VR_WARNING << "Robot definition expects attribute 'type'" << endl;
            robotType = "not set";
        }
        else
        {
            robotType = attr->value();
        }

        // check name counter
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (robot_name_counter.find(robotType) != robot_name_counter.end())
            {
                robot_name_counter[robotType] = robot_name_counter[robotType] + 1;
            }
            else
            {
                robot_name_counter[robotType] = 1;
            }
        }

        // STANDARD NAME
        attr = robotXMLNode->first_attribute("standardname", 0, false);

        if (!attr)
        {
            std::stringstream ss;
            {
                std::lock_guard<std::mutex> lock(mutex);

                if (robot_name_counter[robotType] == 1)
                {
                    ss << robotType;    // first one
                }
                else
                {
                    ss << robotType << "_" << (robot_name_counter[robotType] - 1);
                }
            }
            robotName = ss.str();
        }
        else
        {
            robotName = attr->value();
        }

        // Root
        attr = robotXMLNode->first_attribute("rootnode", 0, false);
        THROW_VR_EXCEPTION_IF(!attr, "Robot definition needs attribute 'RootNode'");
        robotRoot = attr->value();

        // build robot
        RobotPtr robo(new Model(robotName, robotType));
        return robo;
    }


    void RobotIO::processRobotChildNodes(rapidxml::xml_node<char>* robotXMLNode,
                                         RobotPtr robo,
                                         const std::string& robotRoot,
                                         const std::string& basePath,
                                         std::map < RobotNodePtr,
                                         std::vector<ChildFromRobotDef> > & childrenFromRobotFilesMap,
                                         std::vector<rapidxml::xml_node<char>* >& robotNodeSetNodes,
                                         std::vector<rapidxml::xml_node<char>* >& endeffectorNodes,
                                         RobotDescription loadMode)
    {
        std::vector<RobotNodePtr> robotNodes;
        std::map< RobotNodePtr, std::vector< std::string > > childrenMap;
        RobotNodePtr rootNode;
        int robotNodeCounter = 0; // used for robotnodes without names

        //std::vector<rapidxml::xml_node<>* > robotNodeSetNodes;
        //std::vector<rapidxml::xml_node<>* > endeffectorNodes;
        rapidxml::xml_node<>* XMLNode = robotXMLNode->first_node(NULL, 0, false);

        while (XMLNode)
        {
            std::string nodeName_ = XMLNode->name();
            std::string nodeName = getLowerCase(XMLNode->name());

            if (nodeName == "robotnode" || nodeName == "jointnode" || nodeName == "transformationnode" || nodeName == "bodynode" || nodeName == "modelnode")
            {
                std::vector< ChildFromRobotDef > childrenFromRobot;
                std::vector< std::string > childrenNames;
                // check for type
               /* RobotNode::RobotNodeType rntype = RobotNode::Generic;

                if (nodeName == "jointnode")
                {
                    rntype = RobotNode::Joint;
                }

                if (nodeName == "bodynode")
                {
                    rntype = RobotNode::Body;
                }

                if (nodeName == "transformationnode")
                {
                    rntype = RobotNode::Transform;
                }*/

                RobotNodePtr n = processRobotNode(XMLNode, robo, basePath, robotNodeCounter, childrenNames, childrenFromRobot, loadMode);

                if (!n)
                {
                    std::string failedNodeName = processNameAttribute(XMLNode);
                    THROW_VR_EXCEPTION("Failed to create robot node " << failedNodeName << endl);
                }
                else
                {
                    // double name check
                    for (unsigned int i = 0; i < robotNodes.size(); i++)
                    {
                        THROW_VR_EXCEPTION_IF((robotNodes[i]->getName() == n->getName()), "At least two RobotNodes with name <" << n->getName() << "> defined in robot definition");
                    }

                    childrenMap[n] = childrenNames;
                    robotNodes.push_back(n);
					// todo
                    //robo->registerRobotNode(n);

                    if (n->getName() == robotRoot)
                    {
                        rootNode = n;
                    }

                    if (!childrenFromRobot.empty())
                    {
                        childrenFromRobotFilesMap[n] = childrenFromRobot;
                    }
                }

                robotNodeCounter++;
            }
            else if (nodeName == "robotnodeset" || nodeName == "modelnodeset")
            {
                robotNodeSetNodes.push_back(XMLNode);
            }
            else if ("endeffector" == nodeName)
            {
                endeffectorNodes.push_back(XMLNode);
            }
            else
            {
                THROW_VR_EXCEPTION("XML node of type <" << nodeName_ << "> is not supported. Ignoring contents..." << endl);
            }

            XMLNode = XMLNode->next_sibling(NULL, 0, false);
        }

        THROW_VR_EXCEPTION_IF(robotNodes.empty(), "No RobotNodes defined in Robot.");
        THROW_VR_EXCEPTION_IF(!rootNode, "Could not find root node <" << robotRoot << ">");

        if (!RobotFactory::initializeRobot(robo, robotNodes, childrenMap, rootNode))
        {
            THROW_VR_EXCEPTION("Error while initializing Robot" << endl);
        }
    }


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



    VirtualRobot::RobotPtr RobotIO::createRobotFromString(const std::string& xmlString, const std::string& basePath, RobotDescription loadMode)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::RobotPtr robot;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* robotXMLNode = doc.first_node("robot", 0, false);

            if (!robotXMLNode)
            {
                robotXMLNode = doc.first_node("model", 0, false);
            }

            robot = processRobot(robotXMLNode, basePath, loadMode);
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


    VirtualRobot::RobotPtr RobotIO::loadRobot(const std::string& xmlFile, RobotDescription loadMode)
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

        VirtualRobot::RobotPtr res = createRobotFromString(robotXML, basePath, loadMode);

        if (!res)
        {
            VR_ERROR << "Error while parsing file " << fullFile << endl;
        }

        res->applyJointValues();
        res->setFilename(xmlFile);
        return res;
    }


    bool RobotIO::saveXML(RobotPtr robot, const std::string& filename, const std::string& basePath, const std::string& modelDir, bool storeEEF, bool storeRNS, bool storeSensors, bool storeModelFiles)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");


        boost::filesystem::path p(basePath);
        boost::filesystem::path fn(filename);
        boost::filesystem::path pModelDir(modelDir);
        boost::filesystem::path fnComplete = boost::filesystem::operator/(p, fn);
        boost::filesystem::path modelDirComplete = boost::filesystem::operator/(p, pModelDir);

        if (boost::filesystem::exists(modelDirComplete) && !boost::filesystem::is_directory(modelDirComplete))
        {
            VR_ERROR << "Could not create model directory (existing & !dir)  " << pModelDir.string() << endl;
            return false;
        }

        if (!boost::filesystem::is_directory(modelDirComplete))
        {
            if (!boost::filesystem::create_directories(modelDirComplete))
            {
                VR_ERROR << "Could not create model dir  " << modelDirComplete.string() << endl;
                return false;
            }
        }

        std::ofstream f(fnComplete.string().c_str());

        if (!f)
        {
            VR_ERROR << "Could not create file " << fnComplete.string() << endl;
            return false;
        }

        std::string xmlRob = robot->toXML(basePath, modelDir, storeEEF, storeRNS, storeSensors);
        f << xmlRob;
        f.close();

        if (storeModelFiles)
        {
            std::vector<RobotNodePtr> nodes = robot->getModelNodes();

			// todo
            for (size_t i = 0; i < nodes.size(); i++)
            {
                //nodes[i]->saveModelFiles(modelDirComplete.string(), true);
            }
        }

        return true;
    }


} // namespace VirtualRobot
