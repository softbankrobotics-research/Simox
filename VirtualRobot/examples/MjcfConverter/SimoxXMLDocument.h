#pragma once

#include <string>
#include <map>

#include <boost/filesystem/path.hpp>

#include <VirtualRobot/Tools/tinyxml2.h>
#include <VirtualRobot/Nodes/RobotNode.h>


namespace VirtualRobot
{
 
class SimoxXMLDocument : private tinyxml2::XMLDocument
{
    using Base = tinyxml2::XMLDocument;
    friend class RobotNodeVisitor;
    
    
public:
    
    SimoxXMLDocument();
    
    void LoadFile(const char* filename);
    void LoadFile(const std::string& filename);
    
    
    bool hasCollisionModelFile(RobotNodePtr robotNode) const;
    bool hasVisualizationFile(RobotNodePtr robotNode) const;
    
    boost::filesystem::path collisionModelFile(RobotNodePtr robotNode) const;
    boost::filesystem::path visualizationFile(RobotNodePtr robotNode) const;
    
    
private:
    
    using NamePathMap = std::map<std::string, boost::filesystem::path>;

    bool hasEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    boost::filesystem::path getEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    
    NamePathMap collisionModelFiles;
    NamePathMap visualizationFiles;
    
    
    
};


class RobotNodeVisitor : public tinyxml2::XMLVisitor
{
public:
    RobotNodeVisitor(SimoxXMLDocument& document);
    
    virtual bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
private:
    SimoxXMLDocument& document;
    
};


}
