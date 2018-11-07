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
    RobotPtr robot = loadInputFile(inputSimoxXmlFile);
    mjcf::DocumentPtr mjcfDoc = convertToMjcf(robot);
    writeOutputFile(*mjcfDoc, outputMjcfFile);
}

RobotPtr MjcfConverter::loadInputFile(const std::string& inputFilename)
{
    try
    {
        RobotPtr robot = RobotIO::loadRobot(inputFilename, RobotIO::eStructure);
        assert(robot);
        return robot;
    }
    catch (const VirtualRobotException&)
    {
        throw; // rethrow
    }
}

mjcf::DocumentPtr MjcfConverter::convertToMjcf(RobotPtr robot)
{
    mjcf::DocumentPtr doc(new mjcf::Document());
    doc->setModelName(robot->getName());
    
    mjcf::Element* worldBody = doc->worldbody();
    assert(worldBody);
    
    RobotNodePtr rootNode = robot->getRootNode();
    assert(rootNode);

    
    std::map<std::string, mjcf::Element*> addedBodys;
    
    // add root
    mjcf::Element* root = doc->addBodyElement(worldBody, rootNode->getName());
    addedBodys[rootNode->getName()] = root;
    assert(root);
    assert(addedBodys[rootNode->getName()]);

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
    
    // needs explicit type for recursion
    std::function<mjcf::Element*(RobotNodePtr)> addNodeBody;
    addNodeBody = [&](RobotNodePtr node)
    {
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
        
        Eigen::Matrix4f tf = node->getTransformationFrom(node->getParent());
        Eigen::Vector3f pos = tf.block<3,1>(0, 3);
        Eigen::Matrix3f oriMat = tf.block<3,3>(0, 0);
        
        Eigen::Quaternionf quat(oriMat);
        
        element = doc->addBodyElement(parent, node->getName(), pos, quat);
        addedBodys[node->getName()] = element;
        
        std::cout << ts() << "---" << std::endl;
        return element;
    };
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        
        addNodeBody(node);
    }

    return doc;    
    
    if (robot->getRootNode())
    {
        robot->getRootNode()->print(true, true);
    }

    cout << endl;

    std::vector<RobotNodeSetPtr> robotNodeSets = robot->getRobotNodeSets();

    if (robotNodeSets.size() > 0)
    {
        cout << "* RobotNodeSets:" << endl;

        std::vector<RobotNodeSetPtr>::iterator iter = robotNodeSets.begin();

        while (iter != robotNodeSets.end())
        {
            cout << "----------------------------------" << endl;
            (*iter)->print();
            iter++;
        }

        cout << endl;
    }

    cout << "******** Robot ********" << endl;
    
    return doc;
}

void MjcfConverter::writeOutputFile(mjcf::Document& mjcfDoc, 
                                    const std::string& outputFilename)
{
    std::cout << "Writing to " << outputFilename << std::endl;
    mjcfDoc.Print();
    //mjcfDoc.SaveFile(outputFilename.c_str());
}
