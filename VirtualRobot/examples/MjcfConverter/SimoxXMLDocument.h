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
    friend class SimoxXMLVisitor;
    
    
public:
    
    SimoxXMLDocument();
    
    void LoadFile(const boost::filesystem::path& path);
    void LoadFile(const std::string& path);
    
    
    bool hasCollisionModelFile(RobotNodePtr robotNode) const;
    bool hasVisualizationFile(RobotNodePtr robotNode) const;
    
    boost::filesystem::path collisionModelFile(RobotNodePtr robotNode) const;
    boost::filesystem::path visualizationFile(RobotNodePtr robotNode) const;
    
    
private:
    
    using NamePathMap = std::map<std::string, boost::filesystem::path>;

    
    bool hasEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    boost::filesystem::path getEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    
    
    boost::filesystem::path inputFilePath;
    
    NamePathMap collisionModelFiles;
    NamePathMap visualizationFiles;
    
    std::vector<boost::filesystem::path> includedFiles;
    
};


class SimoxXMLVisitor : public tinyxml2::XMLVisitor
{
public:
    SimoxXMLVisitor(SimoxXMLDocument& xml);
    
    virtual bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
private:
    SimoxXMLDocument& xml;
    
};


}

