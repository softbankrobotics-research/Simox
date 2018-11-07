#include "MjcfConverter.h"

#include <tinyxml2.h>

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>


using namespace VirtualRobot;
namespace tx = tinyxml2; 


MjcfConverter::MjcfConverter()
{
}

void MjcfConverter::convert(const std::string& inputSimoxXmlFile, const std::string& outputMjcfFile)
{
    loadInputFile(inputSimoxXmlFile);
    convertToMjcf();
    writeOutputFile(outputMjcfFile);
}

void MjcfConverter::loadInputFile(const std::string& inputFilename)
{
    try
    {
        this->robot = RobotIO::loadRobot(inputFilename, RobotIO::eStructure);
        assert(robot);
    }
    catch (const VirtualRobotException&)
    {
        throw; // rethrow
    }
}

void MjcfConverter::convertToMjcf()
{
    document.reset(new mjcf::Document());
    
    document->setModelName(robot->getName());
    
    mjcf::Element* worldBody = document->worldbody();
    assert(worldBody);
    
    RobotNodePtr rootNode = robot->getRootNode();
    assert(rootNode);
    
    addedBodys.clear();
    
    // add root
    mjcf::Element* root = document->addBodyElement(worldBody, rootNode);
    assert(root);
    addedBodys[rootNode->getName()] = root;
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        addNodeBody(node);
    }

    return; 
}

mjcf::Element* MjcfConverter::addNodeBody(RobotNodePtr node)
{
    int tabs = 0;
    
    auto ts = [&tabs]()
    {
        std::stringstream ss;
        for (int i = 0; i < tabs; ++i)
        {
            ss << "  ";
        }
        return ss.str();
    };
    
    std::cout << ts() << "Process node: " << node->getName();
    if (node->getParent())
    {
        std::cout << " (-> " << node->getParent()->getName() << ")";
    }
    else
    {
        std::cout << " (no parent)";
    }
    std::cout << std::endl;
    
    mjcf::Element* element = addedBodys[node->getName()];
    if (element)
    {
        // break recursion
        std::cout << ts() << "- Node " << node->getName() << " already added" << std::endl;
        std::cout << ts() << "---" << std::endl;
        return element;
    }
    
    mjcf::Element* parent = addedBodys[node->getParent()->getName()];
    if (!parent)
    {
        tabs++;
        // recursion to parent until it was found
        parent = addNodeBody(robot->getRobotNode(node->getParent()->getName()));
        tabs--;
    }
    
    std::cout << ts() << "- Add node:   " << node->getName() 
              << " (-> " << node->getParent()->getName() << ")" << std::endl;
    
    element = document->addBodyElement(parent, node);
    addedBodys[node->getName()] = element;
    
    std::cout << ts() << "---" << std::endl;
    return element;
    
}

void MjcfConverter::writeOutputFile(const std::string& outputFilename)
{
    std::cout << "Writing to " << outputFilename << std::endl;
    document->Print();
    //docuemnt->SaveFile(outputFilename.c_str());
}

