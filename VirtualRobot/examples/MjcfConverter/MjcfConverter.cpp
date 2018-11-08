#include "MjcfConverter.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>


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
        std::cout << "ERROR loading XML file: \n" 
                  << inputXML->ErrorName() << "\n"
                  << inputXML->ErrorStr() << "\n";
    }
}



void MjcfConverter::convertToMjcf()
{
    document.reset(new mjcf::Document());
    
    document->setModelName(robot->getName());
    
    gatherCollisionAndVisualizationFiles();

    addNodeBodies();
    
    addNodeBodyMeshes();
    
    document->Print();
    
    
    mergeEmptyBodies();
    
    return; 
}

void MjcfConverter::gatherCollisionAndVisualizationFiles()
{
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
    std::cout << "nodeCollisionModelFiles: " << std::endl;
    for (auto item : nodeCollisionModelFiles)
    {
        std::cout << "| " << item.first << ": " << item.second << std::endl;
    }
    std::cout << "nodeVisualizationFiles: " << std::endl;
    for (auto item : nodeVisualizationFiles)
    {
        std::cout << "| " << item.first << ": " << item.second << std::endl;
    }
}

void MjcfConverter::addNodeBodies()
{
    // add bodys
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
            std::cout << "skipping (" << dstMeshPath.string() << " already exists)";
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

void MjcfConverter::mergeEmptyBodies()
{
    // merge body leaf nodes with parent if they do not have a geom
    
    
    // assume we have leaf
    
    Element* leafBody;
    
    if (leafBody->FirstChildElement("geom"))
    {
        // has geom
        return;
    }
    
    // merge with parent
    // move all children to parent
    
    for (tx::XMLNode* child = leafBody->FirstChild();
         child;
         child = child->NextSibling())
    {
        leafBody->Parent();
        
        
        
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

