
#include "ModelIO.h"
#include "../RobotFactory.h"
#include "../Model/ModelNodeSet.h"
#include "../VirtualRobotException.h"
#include "../Model/Nodes/ModelJointFixed.h"
#include "../Model/JointSet.h"
#include "../Model/LinkSet.h"
#include "../Model/ModelNodeSet.h"
#include "../Model/Nodes/ModelJointPrismatic.h"
#include "../Model/Nodes/ModelJointRevolute.h"
#include "../Model/Nodes/Attachments/ModelNodeAttachment.h"
#include "../Model/Nodes/Attachments/ModelNodeAttachmentFactory.h"
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



    ModelIO::ModelIO()
    {
    }

    ModelIO::~ModelIO()
    {
    }

    bool ModelIO::searchFile(std::string &filename, const std::string &basePath)
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
    bool ModelIO::processSensor(RobotNodePtr rn, rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath)
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


    ModelPtr ModelIO::loadRobotModel(const std::string &xmlFile, ModelIO::RobotDescription loadMode)
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

    ModelPtr ModelIO::createRobotModelFromString(const std::string& xmlString, const std::string& basePath, RobotDescription loadMode)
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
        catch (VirtualRobot::VirtualRobotException& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                               << "Error code:" << e.what() << endl);

            return RobotPtr();
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

        if (loadMode == ModelIO::eCollisionModel)
        {
            // use collision visualization to build main visualization
            // todo
            //robot->createVisualizationFromCollisionModels();
        }

        return robot;
    }

    ModelPtr ModelIO::processModelDescription(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, ModelIO::RobotDescription loadMode)
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

        // load frames
        XMLNode = robotXMLNode->first_node("frame", 0, false);
        while (XMLNode)
        {
            THROW_VR_EXCEPTION_IF(!robot, "Could not process frame due to missing model defintion...");

            std::string filename = XMLNode->value();

            bool fileOK = searchFile(filename, basePath);
            THROW_VR_EXCEPTION_IF(!fileOK, "Could not find file " << filename);

            bool frameOK = ModelIO::loadFrames(robot, filename);
            THROW_VR_EXCEPTION_IF(!frameOK, "Could not parse frame defintion...");

            XMLNode = XMLNode->next_sibling("frame", 0, false);
        }

        // load sensors
        // todo....


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

        // load NodeSets
        XMLNode = robotXMLNode->first_node("nodeset", 0, false);
        while (XMLNode)
        {
            THROW_VR_EXCEPTION_IF(!robot, "Could not process node sets due to missing model defintion...");

            std::string filename = XMLNode->value();

            bool fileOK = searchFile(filename, basePath);
            THROW_VR_EXCEPTION_IF(!fileOK, "Could not find file " << filename);

            bool nsOK = ModelIO::loadNodeSets(robot, filename);
            THROW_VR_EXCEPTION_IF(!nsOK, "Could not parse node set defintion...");

            XMLNode = XMLNode->next_sibling("nodeset", 0, false);
        }


        return robot;
    }

    RobotNodeSetPtr ModelIO::processModelNodeSet(rapidxml::xml_node<char>* setXMLNode, RobotPtr robo, const std::string& robotRootNode, int& robotNodeSetCounter)
    {
        THROW_VR_EXCEPTION_IF(!setXMLNode, "NULL data for setXMLNode");

        std::string nodeSetName;
        std::string rootNodeName;
        std::string tcpName;

        std::string nodeSetType = BaseIO::getLowerCase(setXMLNode->name());

        // get name and root
        rapidxml::xml_attribute<>* attr = setXMLNode->first_attribute();

        while (attr)
        {
            std::string name = getLowerCase(attr->name());

            if (name == "name")
            {
                THROW_VR_EXCEPTION_IF(!nodeSetName.empty(), "Robot node set contains multiple definitions of attribute name. First value of name is: " << nodeSetName);
                nodeSetName = attr->value();
            }
            else if (name == "kinematicroot")
            {
                THROW_VR_EXCEPTION_IF(!rootNodeName.empty(), "Robot node set contains multiple definitions of attribute kinematicroot. First value of kinematicroot is: " << rootNodeName);
                rootNodeName = attr->value();
            }
            else if (name == "tcp")
            {
                THROW_VR_EXCEPTION_IF(!tcpName.empty(), "Robot node set contains multiple definitions of attribute tcp. First value of tcpis: " << tcpName);
                tcpName = attr->value();
            }

            attr = attr->next_attribute();
        }

        if (nodeSetName.empty())
        {
            std::stringstream ss;
            ss << robo->getType() << "_RobotNodeSet_" << robotNodeSetCounter;
            nodeSetName = ss.str();
            robotNodeSetCounter++;
            VR_WARNING << "RobotNodeSet definition expects attribute 'name'. Setting name to " << nodeSetName << endl;
        }

        if (rootNodeName.empty())
        {
            rootNodeName = robotRootNode;
        }

        std::vector<RobotNodePtr> nodeList;
        processNodeList(setXMLNode, robo, nodeList);

        ModelNodePtr kinRoot;

        if (!rootNodeName.empty())
        {
            kinRoot = robo->getModelNode(rootNodeName);
        }

        FramePtr tcp;

        if (!tcpName.empty())
        {
            tcp = robo->getFrame(tcpName);
        }


        ModelNodeSetPtr rns;
        if (nodeSetType == "jointset")
            rns = JointSet::createJointSet(robo, nodeSetName, nodeList, kinRoot, tcp, true);
        else if (nodeSetType == "linkset")
            rns = LinkSet::createLinkSet(robo, nodeSetName, nodeList, kinRoot, tcp, true);
        else
            rns = ModelNodeSet::createModelNodeSet(robo, nodeSetName, nodeList, kinRoot, tcp, true);

        return rns;
    }


    VisualizationNodePtr ModelIO::processVisualizationTag(rapidxml::xml_node<char>* visuXMLNode, const std::string& tagName, const std::string& basePath, bool& useAsColModel)
    {
        bool enableVisu = true;
        bool coordAxis = false;
        float coordAxisFactor = 1.0f;
        std::string coordAxisText = "";
        std::string visuCoordType = "";
        useAsColModel = false;
        std::string visuFileType = "";
        rapidxml::xml_attribute<>* attr;
        std::vector<Primitive::PrimitivePtr> primitives;
        VisualizationNodePtr visualizationNode;
        std::vector<VisualizationNodePtr> visualizationNodes;

        if (!visuXMLNode)
        {
            return visualizationNode;
        }

        attr = visuXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableVisu = isTrue(attr->value());
        }

        attr = visuXMLNode->first_attribute("useascollisionmodel", 0, false);

        if (attr)
        {
            useAsColModel = isTrue(attr->value());
        }

        if (enableVisu)
        {
            visualizationNodes = processVisuFiles(visuXMLNode, basePath, visuFileType);
            primitives = processPrimitives(visuXMLNode);
            THROW_VR_EXCEPTION_IF(primitives.size() != 0 && visualizationNodes.size() != 0, "Multiple visualization sources defined (file and primitives)" << endl);

            if (visualizationNodes.size() == 1)
            {
                visualizationNode = visualizationNodes.at(0);
            }
            else if (visualizationNodes.size() > 1)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->createUnitedVisualization(visualizationNodes);
            }

            else if (primitives.size() != 0)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->getVisualizationFromPrimitives(primitives);
            }


            rapidxml::xml_node<>* coordXMLNode = visuXMLNode->first_node("coordinateaxis", 0, false);

            if (coordXMLNode)
            {
                attr = coordXMLNode->first_attribute("enable", 0, false);

                if (attr)
                {
                    coordAxis = isTrue(attr->value());
                }

                if (coordAxis)
                {

                    coordAxisFactor = getOptionalFloatByAttributeName(coordXMLNode, "scaling", 1.0f);

                    attr = coordXMLNode->first_attribute("text", 0, false);

                    if (attr)
                    {
                        coordAxisText = attr->value();
                    }
                    else
                    {
                        coordAxisText = tagName;
                    }

                    attr = coordXMLNode->first_attribute("type", 0, false);

                    //THROW_VR_EXCEPTION_IF(!attr, "Missing 'type' attribute in <CoordinateAxis> tag of node " << tagName << "." << endl)
                    if (!attr)
                    {
                        VisualizationFactoryPtr f = VisualizationFactory::first(NULL);

                        if (f)
                        {
                            visuCoordType = f->getDescription();
                        }
                        else
                        {
                            VR_WARNING << "No visualization present..." << endl;
                        }
                    }
                    else
                    {
                        visuCoordType = attr->value();
                    }

                    getLowerCase(visuCoordType);
                    VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(visuCoordType, NULL);

                    if (!visualizationNode)
                    {
                        // create dummy visu
                        if (visualizationFactory)
                        {
                            visualizationNode = visualizationFactory->createVisualization();
                        }
                        else
                        {
                            VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;
                        }
                    }
                    else
                    {
                        THROW_VR_EXCEPTION_IF(visuCoordType.compare(visuFileType) != 0, "Different 'type' attributes not supported for <CoordinateAxis> tag and <File> tag of node " << tagName << "." << endl);
                    }
                    /*
                    if (visualizationNode && visualizationFactory)
                    {
                        VisualizationNodePtr coordVisu = visualizationFactory->createCoordSystem(coordAxisFactor, &coordAxisText);
                        //visualizationNode->attachVisualization("CoordinateSystem", coordVisu);
                        //visualizationNode->showCoordinateSystem(true,coordAxisFactor,&coordAxisText);
                    }
                    else
                    {
                        VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;
                    }*/

                }
            }

            rapidxml::xml_node<>* useColModel = visuXMLNode->first_node("useascollisionmodel", 0, false);

            if (useColModel)
            {
                useAsColModel = true;
            }
        }

        return visualizationNode;
    }

    CollisionModelPtr ModelIO::processCollisionTag(rapidxml::xml_node<char>* colXMLNode, const std::string& tagName, const std::string& basePath)
    {
        rapidxml::xml_attribute<>* attr;
        std::string collisionFileType = "";
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        std::vector<Primitive::PrimitivePtr> primitives;
        std::vector<VisualizationNodePtr> visuNodes;
        bool enableCol = true;

        attr = colXMLNode->first_attribute("enable", 0, false);

        if (attr)
        {
            enableCol = isTrue(attr->value());
        }

        if (enableCol)
        {

            visuNodes = processVisuFiles(colXMLNode, basePath, collisionFileType);
            primitives = processPrimitives(colXMLNode);
            THROW_VR_EXCEPTION_IF(primitives.size() != 0 && visuNodes.size() != 0, "Multiple collision model sources defined (file and primitives)" << endl);

            if (visuNodes.size() != 0)
            {
                if (visuNodes.size() == 1)
                {
                    visualizationNode = visuNodes.at(0);
                }
                else
                {
                    VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(collisionFileType, NULL);

                    if (visualizationFactory)
                    {
                        visualizationNode = visualizationFactory->createUnitedVisualization(visuNodes);
                    }
                    else
                    {
                        VR_WARNING << "VisualizationFactory of type '" << collisionFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;
                    }
                }
            }
            else if (primitives.size() != 0)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
                visualizationNode = visualizationFactory->getVisualizationFromPrimitives(primitives);
            }

            if (visualizationNode)
            {
                std::string colModelName = tagName;
                colModelName += "_ColModel";
                // todo: ID?
                collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
            }
        }

        return collisionModel;
    }

    std::vector<VisualizationNodePtr> ModelIO::processVisuFiles(rapidxml::xml_node<char>* visualizationXMLNode, const std::string& basePath, std::string& fileType)
    {
        rapidxml::xml_node<>* node = visualizationXMLNode;
        std::vector<VisualizationNodePtr> result;
        bool bbox = false;

        if (!node)
        {
            return result;
        }

        rapidxml::xml_node<>* visuFileXMLNode = node->first_node("file", 0, false);

        while (visuFileXMLNode)
        {
            std::string visuFile = "";
            std::string tmpFileType = "";

            rapidxml::xml_attribute<>* attr = visuFileXMLNode->first_attribute("type", 0, false);

            if (!attr)
            {
                if (VisualizationFactory::first(NULL))
                {
                    tmpFileType = VisualizationFactory::first(NULL)->getDescription();
                    getLowerCase(tmpFileType);
                }
                else
                {
                    VR_WARNING << "No visualization present..." << endl;
                }
            }
            else
            {
                tmpFileType = attr->value();
                getLowerCase(tmpFileType);
            }

            if (fileType == "")
            {
                fileType = tmpFileType;
            }

            attr = visuFileXMLNode->first_attribute("boundingbox", 0, false);

            if (attr)
            {
                bbox = isTrue(attr->value());
            }

            getLowerCase(fileType);
            visuFile = processFileNode(visuFileXMLNode, basePath);

            if (visuFile != "")
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(fileType, NULL);

                if (visualizationFactory)
                {
                    if (tmpFileType == fileType)
                    {
                        result.push_back(visualizationFactory->getVisualizationFromFile(visuFile, bbox));
                    }
                    else
                    {
                        VR_WARNING << "Ignoring data from " << visuFileXMLNode->value() << ": visualization type does not match to data from before." << endl;
                    }
                }
                else
                {
                    VR_WARNING << "VisualizationFactory of type '" << fileType << "' not present. Ignoring Visualization data from " << visuFileXMLNode->value() << endl;
                }
            }

            visuFileXMLNode = visuFileXMLNode->next_sibling("file", 0, false);
        }

        return result;
    }

    std::vector<Primitive::PrimitivePtr> ModelIO::processPrimitives(rapidxml::xml_node<char>* primitivesXMLNode)
    {
        std::vector<Primitive::PrimitivePtr> result;
        rapidxml::xml_node<>* node;

        if (!primitivesXMLNode)
        {
            return result;
        }

        if (primitivesXMLNode->name() == std::string{"primitives"})
        {
            node = primitivesXMLNode;
        }
        else
        {
            node = primitivesXMLNode->first_node("primitives", 0, false);
        }

        if (!node)
        {
            return result;
        }

        rapidxml::xml_node<>* primitiveXMLNode = node->first_node();

        while (primitiveXMLNode)
        {
            std::string pName = primitiveXMLNode->name();
            getLowerCase(pName);
            Primitive::PrimitivePtr primitive;

            float lenFactor = 1.f;

            if (hasUnitsAttribute(primitiveXMLNode))
            {
                Units u = getUnitsAttribute(primitiveXMLNode, Units::eLength);

                if (u.isMeter())
                {
                    lenFactor = 1000.f;
                }
            }

            if (pName == "box")
            {
                Primitive::Box* box = new Primitive::Box;
                box->width = convertToFloat(primitiveXMLNode->first_attribute("width", 0, false)->value()) * lenFactor;
                box->height = convertToFloat(primitiveXMLNode->first_attribute("height", 0, false)->value()) * lenFactor;
                box->depth = convertToFloat(primitiveXMLNode->first_attribute("depth", 0, false)->value()) * lenFactor;
                primitive.reset(box);
            }
            else if (pName == "sphere")
            {
                Primitive::Sphere* sphere = new Primitive::Sphere;
                sphere->radius = convertToFloat(primitiveXMLNode->first_attribute("radius", 0, false)->value()) * lenFactor;
                primitive.reset(sphere);
            }
            else if (pName == "cylinder")
            {
                Primitive::Cylinder* cylinder = new Primitive::Cylinder;
                cylinder->radius = convertToFloat(primitiveXMLNode->first_attribute("radius", 0, false)->value()) * lenFactor;
                cylinder->height = convertToFloat(primitiveXMLNode->first_attribute("height", 0, false)->value()) * lenFactor;
                primitive.reset(cylinder);
            }
            else
            {
                VR_ERROR << "Unknown primitive type \"" << pName << "\"; skipping";
            }

            if (primitive)
            {
                Eigen::Matrix4f transform;
                processTransformNode(primitiveXMLNode->first_node("transform", 0, false), "transform", transform);
                primitive->transform = transform;
                result.push_back(primitive);
            }

            primitiveXMLNode = primitiveXMLNode->next_sibling();
        }

        return result;
    }

    void ModelIO::processPhysicsTag(rapidxml::xml_node<char>* physicsXMLNode, const std::string& nodeName, ModelLink::Physics& physics)
    {
        THROW_VR_EXCEPTION_IF(!physicsXMLNode, "NULL data for physicsXMLNode in processPhysicsNode()");
        rapidxml::xml_attribute<>* attr;
        rapidxml::xml_node<>* massXMLNode = physicsXMLNode->first_node("mass", 0, false);

        if (massXMLNode)
        {
            physics.massKg = getFloatByAttributeName(massXMLNode, "value");

            if (!hasUnitsAttribute(massXMLNode))
            {
                VR_ERROR << "No units attribute at <" << nodeName << ">" << endl;
            }

            Units unit = getUnitsAttribute(massXMLNode, Units::eWeight);

            if (unit.isGram())
            {
                physics.massKg *= 0.001f;
            }

            if (unit.isTon())
            {
                physics.massKg *= 1000.0f;
            }

        }
        else
        {
            VR_WARNING << "Expecting mass tag for physics node in <" << nodeName << ">." << endl;
            physics.massKg = 0.0f;
        }

        rapidxml::xml_node<>* comXMLNode = physicsXMLNode->first_node("com", 0, false);

        if (comXMLNode)
        {
            attr = comXMLNode->first_attribute("location", 0, false);

            if (attr)
            {
                std::string loc = attr->value();
                getLowerCase(loc);

                if (loc == "visualizationbboxcenter")
                {
                    physics.comLocation = ModelLink::Physics::eVisuBBoxCenter;
                }
                else if (loc == "joint" || loc == "custom")
                {
                    physics.comLocation = ModelLink::Physics::eCustom;
                }
                else
                {
                    THROW_VR_EXCEPTION("Unsupported Physics <CoM> tag attribute:" << loc);
                }
            }
            else
            {
                physics.comLocation = ModelLink::Physics::eCustom;
            }

            if (physics.comLocation == ModelLink::Physics::eCustom)
            {
                physics.localCoM(0) = getOptionalFloatByAttributeName(comXMLNode, "x", 0.0f);
                physics.localCoM(1) = getOptionalFloatByAttributeName(comXMLNode, "y", 0.0f);
                physics.localCoM(2) = getOptionalFloatByAttributeName(comXMLNode, "z", 0.0f);

                if (hasUnitsAttribute(comXMLNode))
                {
                    Units unitCom = getUnitsAttribute(comXMLNode, Units::eLength);

                    if (unitCom.isMeter())
                    {
                        physics.localCoM *= 1000.0f;
                    }

                }
            }

        }

        rapidxml::xml_node<>* inMatXMLNode = physicsXMLNode->first_node("inertiamatrix", 0, false);

        if (inMatXMLNode)
        {
            physics.inertiaMatrix = process3x3Matrix(inMatXMLNode);
            std::vector< Units > unitsAttr = getUnitsAttributes(inMatXMLNode);
            Units uWeight("kg");
            Units uLength("m");

            for (size_t i = 0; i < unitsAttr.size(); i++)
            {
                if (unitsAttr[i].isWeight())
                {
                    uWeight = unitsAttr[i];
                }

                if (unitsAttr[i].isLength())
                {
                    uLength = unitsAttr[i];
                }
            }

            float factor = 1.0f;

            if (uWeight.isGram())
            {
                factor *= 0.001f;
            }

            if (uWeight.isTon())
            {
                factor *= 1000.0f;
            }

            if (uLength.isMillimeter())
            {
                factor *= 0.000001f;
            }

            physics.inertiaMatrix *= factor;

        }
        else
        {
            physics.inertiaMatrix.setZero(); // this will trigger an automatically determination of the inertia matrix during initialization
        }

        rapidxml::xml_node<>* ignoreColXMLNode = physicsXMLNode->first_node("ignorecollision", 0, false);

        while (ignoreColXMLNode)
        {
            rapidxml::xml_attribute<>* attr = ignoreColXMLNode->first_attribute("name", 0, false);
            THROW_VR_EXCEPTION_IF(!attr, "Expecting 'name' attribute in <IgnoreCollision> tag..." << endl)
            std::string s(attr->value());
            physics.ignoreCollisions.push_back(s);
            ignoreColXMLNode = ignoreColXMLNode->next_sibling("ignorecollision", 0, false);
        }

        rapidxml::xml_node<>* simulationtype = physicsXMLNode->first_node("simulationtype", 0, false);

        if (simulationtype)
        {
            rapidxml::xml_attribute<>* attr = simulationtype->first_attribute("value", 0, false);
            THROW_VR_EXCEPTION_IF(!attr, "Expecting 'value' attribute in <SimulationType> tag..." << endl)
            std::string s(attr->value());
            getLowerCase(s);

            if (s == "dynamic" || s == "dynamics")
            {
                physics.simType = VirtualRobot::ModelLink::Physics::eDynamic;
            }
            else if (s == "static")
            {
                physics.simType = VirtualRobot::ModelLink::Physics::eStatic;
            }
            else if (s == "kinematic")
            {
                physics.simType = VirtualRobot::ModelLink::Physics::eKinematic;
            }

            // otherwise eUnknown remains
        }
        rapidxml::xml_node<>* frictionXMLNode = physicsXMLNode->first_node("friction", 0, false);

        if (frictionXMLNode)
        {
            physics.friction = getFloatByAttributeName(frictionXMLNode, "value");
        }
        else
        {
            physics.friction = -1.0f;
        }


    }

    bool ModelIO::createNodeSetsFromString(const RobotPtr &robot, const std::string &xmlString)
    {
        if (!robot)
            return false;

        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);
        int rnsNr = 0;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags

            rapidxml::xml_node<char>* node = doc.first_node();
            while (node)
            {
                std::string nodeName = getLowerCase(node->name());
                if (nodeName == "robotnodeset" || nodeName == "jointset" || nodeName == "linkset" || nodeName == "modelnodeset")
                {
                    // registers rns to robot
                    ModelNodeSetPtr r = ModelIO::processModelNodeSet(node, robot, robot->getRootNode()->getName(), rnsNr);
                    THROW_VR_EXCEPTION_IF(!r, "Invalid ModelNodeSet definition " << endl);
                    rnsNr++;
                }
                node = node->next_sibling();
            }
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse node set in xml definition" << endl
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
            THROW_VR_EXCEPTION("Error while parsing node set xml definition" << endl);
            return false;
        }

        delete[] y;
        return true;
    }

    bool ModelIO::loadNodeSets(const RobotPtr &robot, const std::string &filename)
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
        std::string nsXML(buffer.str());
        in.close();

        return createNodeSetsFromString(robot, nsXML);
    }

    bool ModelIO::createFramesFromString(const RobotPtr &robot, const std::string &xmlString)
    {
        if (!robot)
            return false;

        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);
        int rnsNr = 0;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags

            rapidxml::xml_node<char>* node = doc.first_node();
            while (node)
            {
                std::string nodeName = getLowerCase(node->name());
                if (nodeName == "frame")
                {
                    // registers rns to robot
                    FramePtr r = ModelIO::processFrame(node, robot);
                    THROW_VR_EXCEPTION_IF(!r, "Invalid Frame definition " << endl);
                }
                node = node->next_sibling();
            }
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse frame in xml definition" << endl
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
            THROW_VR_EXCEPTION("Error while parsing frame xml definition" << endl
                               << "Error code:" << e.what() << endl);
            return false;
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing frame xml definition" << endl);
            return false;
        }

        delete[] y;
        return true;
    }

    bool ModelIO::loadFrames(const RobotPtr &robot, const std::string &filename)
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
        std::string nsXML(buffer.str());
        in.close();

        return createFramesFromString(robot, nsXML);
    }



    FramePtr ModelIO::processFrame(rapidxml::xml_node<char>* frameXMLNode, RobotPtr robo)
    {
        THROW_VR_EXCEPTION_IF(!frameXMLNode, "NULL data in processRobotNode");

        // get name
        std::string frameName = BaseIO::processNameAttribute(frameXMLNode);

        THROW_VR_EXCEPTION_IF (frameName.empty(), "Frame node must conatin a name attribute");

        ModelNodePtr parentNode;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

        rapidxml::xml_node<>* node = frameXMLNode->first_node();

        while (node)
        {
            std::string nodeName = BaseIO::getLowerCase(node->name());

            if (nodeName == "transform")
            {
                BaseIO::processTransformNode(frameXMLNode, frameName, transformMatrix);
            } else if (nodeName == "parent")
            {
                THROW_VR_EXCEPTION_IF(parentNode, "Only one parent allowed...");
                std::string nameStr("node");
                std::string parentNodeName = BaseIO::processStringAttribute(nameStr, node, false);
                THROW_VR_EXCEPTION_IF(parentNodeName.empty(), "Parent name must not be empty");
                THROW_VR_EXCEPTION_IF(!robo->hasModelNode(parentNodeName), "Robot does not node model node with name " + parentNodeName + " in frame " + frameName);
                parentNode = robo->getModelNode(parentNodeName);
            } else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Frame <" << frameName << ">." << endl);
            }

            node = node->next_sibling();
        }
        THROW_VR_EXCEPTION_IF(!parentNode, "No parent given in frame " << frameName);

        THROW_VR_EXCEPTION_IF(robo->hasFrame(frameName), "Frame names must be unique. Frame with name " << frameName << " already present in robot " << robo->getName());
        ModelNodeAttachmentFactoryPtr mff = ModelNodeAttachmentFactory::fromName("ModelFrame", NULL);
        THROW_VR_EXCEPTION_IF(!mff, "Could not instanciate model frame factory...");

        mff->createAttachment(frameName, transformMatrix, VisualizationNodePtr());


        return FramePtr();
    }

} // namespace VirtualRobot
