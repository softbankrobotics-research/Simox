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
    MjcfDocumentPtr mjcfDoc = convertToMjcf(robot);
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

MjcfDocumentPtr MjcfConverter::convertToMjcf(RobotPtr robot)
{
    MjcfDocumentPtr doc(new MjcfDocument());
    
    tx::XMLElement* elMujoco = doc->NewElement("mujoco");
    doc->InsertEndChild(elMujoco);
    
    
    
    
    /*
    cout << "******** Robot ********" << endl;
    cout << "* Name: " << robot->getName() << endl;
    cout << "* Type: " << robot->getType() << endl;

    if (robot->getRootNode())
    {
        cout << "* Root Node: " << robot->getRootNode()->getName() << endl;
    }
    else
    {
        cout << "* Root Node: not set" << endl;
    }

    cout << endl;

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
    */
    
    return doc;
}

void MjcfConverter::writeOutputFile(MjcfDocument& mjcfDoc, 
                                    const std::string& outputFilename)
{
    std::cout << "Writing to " << outputFilename << std::endl;
    mjcfDoc.Print();
    mjcfDoc.SaveFile(outputFilename.c_str());
}
