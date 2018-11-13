#include "MjcfConverter.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "utils.h"
#include "xml_visitors.h"



using namespace VirtualRobot;
namespace tx = tinyxml2; 
namespace fs = boost::filesystem;
using Element = mjcf::Element;


MjcfConverter::MjcfConverter() :
    masslessBodySanitizer(document, robot)
{
}

void MjcfConverter::convert(const std::string& inputSimoxXmlFile, 
                            const std::string& outputDirectory)
{
    setPaths(inputSimoxXmlFile, outputDirectory);
    
    loadInputFile();
    
    document.reset(new mjcf::Document());
    
    document->setModelName(robot->getName());
    document->compiler()->SetAttribute("angle", "radian");
    document->compiler()->SetAttribute("balanceinertia", "true");
    
    makeEnvironment();
    
    std::cout << "Creating bodies structure ..." << std::endl;
    addNodeBodies();
    
    std::cout << "Adding meshes and geoms ..." << std::endl;
    addNodeBodyMeshes();
    
    std::cout << "===========================" << std::endl
              << "Current model: "             << std::endl
              << "--------------"              << std::endl;
    document->Print();
    std::cout << "===========================" << std::endl;
    
    std::cout << "Merging massless bodies ..." << std::endl;
    masslessBodySanitizer.sanitize();
    
    std::cout << "Adding contact excludes ..." << std::endl;
    addContactExcludes();

    std::cout << "Adding actuators ..." << std::endl;
    addActuators();
    
    std::cout << "Done.";
    
    std::cout << std::endl;
    std::cout << "===========================" << std::endl
              << "Output file: "             << std::endl
              << "------------"              << std::endl;
    document->Print();
    std::cout << "===========================" << std::endl;
    
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
    
    std::cout << "Loading robot via RobotIO: " << inputFilePath << std::endl;
    try
    {
        robot = RobotIO::loadRobot(inputFilePath.string(), RobotIO::eFull);
        assert(robot);
    }
    catch (const VirtualRobotException&)
    {
        throw; // rethrow
    }
}

void MjcfConverter::writeOutputFile()
{
    assert(!outputFileName.empty());
    std::cout << "Writing to " << outputFileName << std::endl;
    document->SaveFile(outputFileName.c_str());
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
        VisualizationNodePtr visualization = node->getVisualization(SceneObject::VisualizationType::Full);
        
        if (!visualization)
        {
            continue;
        }
        
        
        std::cout << "Node " << node->getName() << ":\t";
        
        fs::path srcMeshPath = visualization->getFilename();
        
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

void MjcfConverter::addContactExcludes()
{
    RobotNodePtr rootNode = robot->getRootNode();
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        for (std::string& ignore : node->getPhysics().ignoreCollisions)
        {
            
        }
    }
}

void MjcfConverter::addActuators()
{
    std::vector<const mjcf::Element*> jointElements = getAllElements("joint");
    
    for (auto joint : jointElements)
    {
        std::string name = joint->Attribute("name");
        document->addMotorElement(name);
    }
}

std::vector<const mjcf::Element*> MjcfConverter::getAllElements(const std::string& elemName)
{
    mjcf::ListElementsVisitor visitor(elemName);
    document->worldbody()->Accept(&visitor);
    return visitor.getFoundElements();
}





