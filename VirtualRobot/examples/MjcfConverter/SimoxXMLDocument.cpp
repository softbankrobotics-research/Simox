#include "SimoxXMLDocument.h"

#include "exceptions.h"
#include "utils.h"


using namespace VirtualRobot;
using namespace tinyxml2;

namespace fs = boost::filesystem;


SimoxXMLDocument::SimoxXMLDocument()
{
}

void SimoxXMLDocument::LoadFile(const fs::path& path)
{
    this->inputFilePath = path;
    
    int errID = Base::LoadFile(path.c_str());
    if (errID)
    {
        throw MjcfXmlLoadFileFailed(ErrorName(), ErrorStr());
    }
    
    collisionModelFiles.clear();
    visualizationFiles.clear();
    includedFiles.clear();
    
    XMLNode* xmlRobot = FirstChildElement("Robot");
    assert(xmlRobot);

    SimoxXMLVisitor visitor(*this);
    xmlRobot->Accept(&visitor);
    
    for (fs::path file : includedFiles)
    {
        SimoxXMLDocument includedXML;
        includedXML.LoadFile(file);
        
        auto copyMap = [](const NamePathMap& src, NamePathMap& dst)
        {
            for (auto item : src)
            {
                dst[item.first] = item.second;
            }
        };
        
        copyMap(includedXML.collisionModelFiles, this->collisionModelFiles);
        copyMap(includedXML.visualizationFiles, this->visualizationFiles);
    }
}

void SimoxXMLDocument::LoadFile(const std::string& filename)
{
    LoadFile(fs::path(filename));
}

bool SimoxXMLDocument::hasCollisionModelFile(RobotNodePtr robotNode) const
{
    return hasEntry(robotNode, collisionModelFiles);
}

bool SimoxXMLDocument::hasVisualizationFile(RobotNodePtr robotNode) const
{
    return hasEntry(robotNode, visualizationFiles);
}

fs::path SimoxXMLDocument::collisionModelFile(RobotNodePtr robotNode) const
{
    return getEntry(robotNode, collisionModelFiles);
}

fs::path SimoxXMLDocument::visualizationFile(RobotNodePtr robotNode) const
{
    return getEntry(robotNode, visualizationFiles);
}

bool SimoxXMLDocument::hasEntry(RobotNodePtr robotNode, const NamePathMap& map) const
{
    return map.find(robotNode->getName()) != map.end();
}

fs::path SimoxXMLDocument::getEntry(
        RobotNodePtr robotNode, const SimoxXMLDocument::NamePathMap& map) const
{
    auto item = map.find(robotNode->getName());
    if (item == map.end())
    {
        return fs::path("");
    }
    return item->second;
}


SimoxXMLVisitor::SimoxXMLVisitor(SimoxXMLDocument& document) : 
    xml(document)
{
}

bool SimoxXMLVisitor::VisitEnter(const tinyxml2::XMLElement& elem, const tinyxml2::XMLAttribute*)
{
    if (isElement(&elem, "RobotNode"))
    {
        std::string nodeName = elem.Attribute("name");
        
        const XMLElement* xmlVisu = elem.FirstChildElement("Visualization");
        const XMLElement* xmlColModel = elem.FirstChildElement("CollisionModel");

        if (xmlColModel)
        {
            std::string filename = xmlColModel->FirstChildElement("File")->GetText();
            xml.collisionModelFiles[nodeName] = filename;
        }
        
        if (xmlVisu)
        {
            std::string filename = xmlVisu->FirstChildElement("File")->GetText();
            xml.visualizationFiles[nodeName] = filename;
        }

    }
    else if (isElement(&elem, "ChildFromRobot"))
    {
        const XMLElement* file = elem.FirstChildElement("File");
        assert(file);
        fs::path relPath = file->GetText();
        
        xml.includedFiles.push_back(xml.inputFilePath.parent_path() / relPath);
    }
    
    return true;
}
