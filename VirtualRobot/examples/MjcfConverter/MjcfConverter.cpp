#include "MjcfConverter.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "utils.h"


using namespace VirtualRobot;
namespace tx = tinyxml2; 
namespace fs = boost::filesystem;
using Element = mjcf::Element;


MjcfConverter::MjcfConverter()
{
}

void MjcfConverter::convert(const std::string& inputSimoxXmlFile, 
                            const std::string& outputDirectory)
{
    setPaths(inputSimoxXmlFile, outputDirectory);
    
    loadInputFile();
    convertToMjcf();
    writeOutputFile();
}

void MjcfConverter::setPaths(const std::string& inputFilename, 
                             const std::string& outputDirectory)
{
    this->inputFilePath = inputFilename;
    
    inputFileDirectory = inputFilePath.parent_path();
    inputFileName = inputFilePath.filename();
    
    this->outputDirectory = outputDirectory;
    outputFileName = this->outputDirectory / inputFileName;
    
    outputMeshRelDirectory = "mesh";
    
    
    auto ensureDirExists = [](const fs::path& path)
    {
        if (!fs::exists(path))
        {
            fs::create_directory(path);
        }
    };
    
    ensureDirExists(outputDirectory);
    ensureDirExists(outputDirectory / outputMeshRelDirectory);
    
    assert(!inputFileDirectory.empty());
}

void MjcfConverter::makeEnvironment()
{
    document->addSkyboxTexture(Eigen::Vector3f(.8f, .9f, .95f), 
                               Eigen::Vector3f(.4f, .6f, .8f));
}

void MjcfConverter::loadInputFile()
{
    assert(!inputFilePath.empty());
    
    try
    {
        this->robot = RobotIO::loadRobot(inputFilePath.string(), RobotIO::eStructure);
        assert(robot);
    }
    catch (const VirtualRobotException&)
    {
        throw; // rethrow
    }
    
    inputXML.reset(new tx::XMLDocument());
    
    if (inputXML->LoadFile(inputFilePath.c_str()))
    {
        throw MjcfXmlLoadFileFailed(inputXML->ErrorName(), inputXML->ErrorStr());
    }
}

void MjcfConverter::writeOutputFile()
{
    assert(!outputFileName.empty());
    
    std::cout << std::endl;
    document->Print();
    
    std::cout << "Writing to " << outputFileName << std::endl;
    document->SaveFile(outputFileName.c_str());
}


void MjcfConverter::convertToMjcf()
{
    document.reset(new mjcf::Document());
    
    document->setModelName(robot->getName());
    document->compiler()->SetAttribute("angle", "radian");
    
    makeEnvironment();
    
    std::cout << "Collecting collision model and visualization files..." << std::endl;
    gatherCollisionAndVisualizationFiles();

    std::cout << "Creating bodies structure ..." << std::endl;
    addNodeBodies();
    
    std::cout << "Adding meshes and geoms ..." << std::endl;
    addNodeBodyMeshes();
    
    std::cout << "===========================" << std::endl
              << "Current model: "             << std::endl
              << "--------------"              << std::endl;
    document->Print();
    std::cout << "===========================" << std::endl;
    
    std::cout << "Merging empty bodies ..." << std::endl;
    sanitizeMasslessBodies();
    
    std::cout << "Adding contact excludes ..." << std::endl;
    document->addContactExcludes(nodeBodies[robot->getRootNode()->getName()]);
    
    std::cout << "Done.";
    
    return; 
}

void MjcfConverter::gatherCollisionAndVisualizationFiles()
{
    nodeCollisionModelFiles.clear();
    nodeVisualizationFiles.clear();
    
    tx::XMLNode* xmlRobot = inputXML->FirstChildElement("Robot");
    assert(xmlRobot);
    
    for (tx::XMLElement* xmlRobotNode = xmlRobot->FirstChildElement("RobotNode");
         xmlRobotNode;
         xmlRobotNode = xmlRobotNode->NextSiblingElement("RobotNode"))
    {
        std::string nodeName = xmlRobotNode->Attribute("name");
        
        tx::XMLElement* xmlVisu = xmlRobotNode->FirstChildElement("Visualization");
        tx::XMLElement* xmlColModel = xmlRobotNode->FirstChildElement("CollisionModel");
        
        if (xmlVisu)
        {
            std::string filename = xmlVisu->FirstChildElement("File")->GetText();
            nodeVisualizationFiles[nodeName] = filename;
        }
        
        if (xmlColModel)
        {
            std::string filename = xmlColModel->FirstChildElement("File")->GetText();
            nodeCollisionModelFiles[nodeName] = filename;
        }
    }
}

void MjcfConverter::addNodeBodies()
{
    nodeBodies.clear();
    
    RobotNodePtr rootNode = robot->getRootNode();
    assert(rootNode);
    
    // add root
    Element* root = document->addBodyElement(document->worldbody(), rootNode);
    nodeBodies[rootNode->getName()] = root;
    assert(root);
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        addNodeBody(node);
    }
}

void MjcfConverter::addNodeBodyMeshes()
{
    bool meshlabserverAviable = system("which meshlabserver > /dev/null 2>&1") == 0;
    bool notAvailableReported = false;
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        auto item = nodeVisualizationFiles.find(node->getName());
        if (item == nodeVisualizationFiles.end())
        {
            continue;
        }
        
        std::cout << "Node " << node->getName() << ":\t";
        
        fs::path srcMeshPath = item->second;
        
        if (srcMeshPath.is_relative())
        {
            // resolve relative path
            srcMeshPath = inputFileDirectory / srcMeshPath;
        }
        
        fs::path dstMeshFileName = srcMeshPath.filename();
        dstMeshFileName.replace_extension("stl");
        fs::path dstMeshRelPath = outputMeshRelDirectory / dstMeshFileName;
        fs::path dstMeshPath = outputDirectory / dstMeshRelPath;
        
        if (!fs::exists(dstMeshPath))
        {
            if (srcMeshPath.extension().string() != "stl")
            {
                std::cout << "Converting to .stl: " << srcMeshPath;
                
                if (!meshlabserverAviable)
                {
                    if (!notAvailableReported)
                    {
                        std::cout << std::endl 
                                  << "Command 'meshlabserver' not available, "
                                     "cannot convert meshes." << std::endl;
                        notAvailableReported = true;
                    }
                    continue;
                }
                
                // meshlabserver available
                std::stringstream convertCommand;
                convertCommand << "meshlabserver"
                               << " -i " << srcMeshPath.string() 
                               << " -o " << dstMeshPath.string();
                
                // run command
                int r = system(convertCommand.str().c_str());
                if (r != 0)
                {
                    std::cout << "Command returned with error: " << r << "\n"
                              << "Command was: " << convertCommand.str() << std::endl;
                }
            }
            else
            {
                std::cout << "Copying: " << srcMeshPath << "\n"
                          << "     to: " << dstMeshPath;
                fs::copy_file(srcMeshPath, dstMeshPath);
            }
        }
        else
        {
            std::cout << "skipping (" << dstMeshRelPath.string() << " already exists)";
        }
        std::cout << std::endl;
        
        
        
        // add asset
        std::string meshName = node->getName();
        document->addMeshElement(meshName, dstMeshRelPath.string());
        
        // add geom to body
        Element* body = nodeBodies[node->getName()];
        document->addGeomElement(body, meshName);
    }
}



Element* MjcfConverter::addNodeBody(RobotNodePtr node)
{
    Element* element = nodeBodies[node->getName()];
    if (element)
    {
        // break recursion
        return element;
    }
    
    Element* parent = nodeBodies[node->getParent()->getName()];
    if (!parent)
    {
        parent = addNodeBody(robot->getRobotNode(node->getParent()->getName()));
    }
    
    element = document->addBodyElement(parent, node);
    nodeBodies[node->getName()] = element;

    return element;
}

void MjcfConverter::sanitizeMasslessBodies()
{
    // merge body leaf nodes with parent if they do not have a geom
    
    // assume we have leaf
    
    Element* root = document->worldbody()->FirstChildElement("body");
    
    for (Element* body = root->FirstChildElement("body");
         body;
         body = body->NextSiblingElement("body"))
    {
        sanitizeMasslessBodyRecursion(body);
    }
}

void MjcfConverter::sanitizeMasslessBodyRecursion(mjcf::Element* body)
{
    assertElementIsBody(body);
    
    std::cout << "- Node '" << body->Attribute("name") << "': " << std::endl;
    
    // leaf => end of recursion
    if (!hasElement(body, "body"))
    {
        std::cout << "  | Leaf";
        if (!hasMass(body))
        {
            std::cout << " without mass" << std::endl;
            sanitizeMasslessLeafBody(body);
        }
        else
        {
            std::cout << std::endl;
        }
        return; 
    }
    
    // non-leaf body
    std::cout << "  | Non-leaf";
    
    if (!hasMass(body))
    {
        std::cout << " without mass" << std::endl;
        
        // check whether there is only one child body
        Element* childBody = body->FirstChildElement("body");
        if (!childBody->NextSiblingElement("body"))
        {
            std::string bodyName = body->Attribute("name");
            std::string childBodyName = childBody->Attribute("name");
            
            std::cout << "  | Single child body => merging '" << childBodyName
                      << "' into '" << bodyName << "'" << std::endl;
            
            // merge childBody into body => move all its elements here
            for (tx::XMLNode* grandChild = childBody->FirstChild(); grandChild;
                 grandChild = grandChild->NextSibling())
            {
                std::cout << "  |  | Moving '" << grandChild->Value() << "'" << std::endl;
                
                // clone grandchild
                tx::XMLNode* grandChildClone = grandChild->DeepClone(nullptr);
                
                // insert to body
                body->InsertEndChild(grandChildClone);
            }
            
            // check child for pose attributes
            if (childBody->Attribute("pos") || childBody->Attribute("quat"))
            {
                // update body's transform
                // get tf from robot node
                
                RobotNodePtr bodyNode = robot->getRobotNode(bodyName);
                RobotNodePtr childNode = robot->getRobotNode(childBodyName);
                
                Eigen::Matrix4f bodyPose = bodyNode->getTransformationFrom(bodyNode->getParent());
                Eigen::Matrix4f childPose = childNode->getTransformationFrom(childNode->getParent());
                
                bodyPose = bodyPose * childPose;
                document->setBodyPose(body, bodyPose);
                
                // adapt axis of any joint in body
                
                for (Element* joint = body->FirstChildElement("joint"); joint;
                     joint = joint->NextSiblingElement("joint"))
                {
                    Eigen::Vector3f axis = strToVec(joint->Attribute("axis"));
                    // apply child orientation
                    axis = childPose.block<3,3>(0, 0) * axis;
                    document->setJointAxis(joint, axis);
                }
            }
            
            
            // update body name
            std::size_t prefixSize = commonPrefixLength(bodyName, childBodyName);
            
            std::stringstream newName;
            newName << bodyName.substr(0, prefixSize)
                    << bodyName.substr(prefixSize) << "~" << childBodyName.substr(prefixSize);            
            body->SetAttribute("name", newName.str().c_str());
            
            
            // delete child
            body->DeleteChild(childBody);
        }
    }
    else
    {
        std::cout << std::endl;
    }
    
    for (Element* child = body->FirstChildElement("body");
         child;
         child = child->NextSiblingElement("body"))
    {
        sanitizeMasslessBodyRecursion(child);
    }
    
}

void MjcfConverter::sanitizeMasslessLeafBody(mjcf::Element* body)
{
    
    assert(!hasElement(body, "body"));
    assert(!hasMass(body));
    
    if (body->NoChildren()) // is completely empty?
    {
        // leaf without geom: make it a site
        std::cout << "  | Empty => Changing body '" << body->Attribute("name") << "' to site." << std::endl;
        body->SetValue("site");
    }
    else
    {
        // add a small mass
        std::cout << "  | Not empty => Adding dummy inertial." << std::endl;
        document->addDummyInertial(body);
    }
}





