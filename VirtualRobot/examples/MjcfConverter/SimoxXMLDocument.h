#pragma once

#include <string>
#include <map>

#include <boost/filesystem/path.hpp>

#include <VirtualRobot/Tools/tinyxml2.h>
#include <VirtualRobot/Nodes/RobotNode.h>


namespace VirtualRobot
{

/**
 * @brief A Simox XML document offering access to information not included 
 * in a VirtualRobot::Robot.
 */
class SimoxXMLDocument : private tinyxml2::XMLDocument
{
    using Base = tinyxml2::XMLDocument;
    friend class SimoxXMLVisitor;
    
    
public:
    
    /// Constructor.
    SimoxXMLDocument();
    
    /**
     * @brief Load a Simox XML file.
     * Loads the given XML file and traverses it to collect the wanted 
     * information. If the file references any other XML files via a
     * <ChildFromRobot> element, loads that as well and aggregates its
     * information into this.
     */
    void LoadFile(const boost::filesystem::path& path);
    /// @see LoadFile(const boost::filesyste::path&)
    void LoadFile(const std::string& path);
    
    /// Indicate whether a RobotNode has specified a collision model file.
    bool hasCollisionModelFile(RobotNodePtr robotNode) const;
    /// Indicate whether a RobotNode has specified a visualization file.
    bool hasVisualizationFile(RobotNodePtr robotNode) const;
    
    /// Get the collision model file of a RobotNode. If the node did not 
    /// specify a collision model file, an empty path is returned.
    boost::filesystem::path collisionModelFile(RobotNodePtr robotNode) const;
    /// Get the visualization file of a RobotNode. If the node did not 
    /// specify a visualization model file, an empty path is returned.
    boost::filesystem::path visualizationFile(RobotNodePtr robotNode) const;
    
    
private:
    
    /// Map from (node) name to a path.
    using NamePathMap = std::map<std::string, boost::filesystem::path>;

    /// Return true when there is an entry for the robot node in the map.
    bool hasEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    /// Return the value of the robot node in the map. If none is found, return an empty path.
    boost::filesystem::path getEntry(RobotNodePtr robotNode, const NamePathMap& map) const;
    
    /// The input file path. Necessary for resolving relative paths.
    boost::filesystem::path inputFilePath;
    
    /// The collision model files of RobotNodes.
    NamePathMap collisionModelFiles;
    /// The visualization files of RobotNodes.
    NamePathMap visualizationFiles;
    
    
    
};


/**
 * @brief The SimoxXMLVisitor class
 */
class SimoxXMLVisitor : public tinyxml2::XMLVisitor
{
    friend class SimoxXMLDocument;
    
public:
    
    SimoxXMLVisitor(SimoxXMLDocument& xml);
    
    /// Stores mesh files of RobotNodes, and stores files included <ChildFromRobot> elements.
    virtual bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
    
private:
    
    SimoxXMLDocument& xml;
    
    /// The files included by the input file.
    std::vector<boost::filesystem::path> includedFiles;
    
};


}

