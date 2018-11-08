#include "SimoxXMLDocument.h"

#include "exceptions.h"
#include "utils.h"


using namespace VirtualRobot;
namespace tx = tinyxml2;


SimoxXMLDocument::SimoxXMLDocument()
{
}

void SimoxXMLDocument::LoadFile(const char* filename)
{
    int errID = Base::LoadFile(filename);
    if (errID)
    {
        throw MjcfXmlLoadFileFailed(ErrorName(), ErrorStr());
    }
    
    collisionModelFiles.clear();
    visualizationFiles.clear();
    
    tx::XMLNode* xmlRobot = FirstChildElement("Robot");
    assert(xmlRobot);

    RobotNodeVisitor visitor(*this);
    xmlRobot->Accept(&visitor);
}


void SimoxXMLDocument::LoadFile(const std::string& filename)
{
    LoadFile(filename.c_str());
}

bool SimoxXMLDocument::hasCollisionModelFile(RobotNodePtr robotNode) const
{
    return hasEntry(robotNode, collisionModelFiles);
}

bool SimoxXMLDocument::hasVisualizationFile(RobotNodePtr robotNode) const
{
    return hasEntry(robotNode, visualizationFiles);
}

boost::filesystem::path SimoxXMLDocument::collisionModelFile(RobotNodePtr robotNode) const
{
    return getEntry(robotNode, collisionModelFiles);
}

boost::filesystem::path SimoxXMLDocument::visualizationFile(RobotNodePtr robotNode) const
{
    return getEntry(robotNode, visualizationFiles);
}

bool SimoxXMLDocument::hasEntry(RobotNodePtr robotNode, const NamePathMap& map) const
{
    return map.find(robotNode->getName()) != map.end();
}

boost::filesystem::path SimoxXMLDocument::getEntry(
        RobotNodePtr robotNode, const SimoxXMLDocument::NamePathMap& map) const
{
    auto item = map.find(robotNode->getName());
    if (item == map.end())
    {
        return boost::filesystem::path("");
    }
    return item->second;
}


RobotNodeVisitor::RobotNodeVisitor(SimoxXMLDocument& document) : 
    document(document)
{
}

bool RobotNodeVisitor::VisitEnter(const tinyxml2::XMLElement& robotNode, const tinyxml2::XMLAttribute*)
{
    if (isElement(&robotNode, "RobotNode"))
    {
        std::string nodeName = robotNode.Attribute("name");
                
        const tx::XMLElement* xmlVisu = robotNode.FirstChildElement("Visualization");
        const tx::XMLElement* xmlColModel = robotNode.FirstChildElement("CollisionModel");
        
        if (xmlVisu)
        {
            std::string filename = xmlVisu->FirstChildElement("File")->GetText();
            document.visualizationFiles[nodeName] = filename;
        }
        
        if (xmlColModel)
        {
            std::string filename = xmlColModel->FirstChildElement("File")->GetText();
            document.collisionModelFiles[nodeName] = filename;
        }
    }
}
