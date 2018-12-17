#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>

#include "MjcfDocument.h"
#include "MasslessBodySanitizer.h"
#include "utils.h"


namespace VirtualRobot
{
namespace mjcf
{

    class MujocoIO
    {
    public:
        
        /// Constructor.
        /// @throws VirtualRobotException if robot is null
        MujocoIO(RobotPtr robot);
        
        /**
         * @brief Create a Mujoco XML (MJCF) document for the given robot.
         * @param filename   the output filename (without directory)
         * @param basePath   the output directory
         * @param meshRelDir the directory relative to basePath where meshes shall be placed
         */
        void saveMJCF(const std::string& filename, const std::string& basePath, 
                      const std::string& meshRelDir);
        
        
        /// Set the scaling for lenghts.
        void setLengthScaling(float value);

        /// Set the actuator type.
        void setActuatorType(ActuatorType value);
        
        /**
         * @brief Enable or disable adding of a mocap body controlling the robot pose.
         * 
         * @param enabled If true:
         * - Add a free joint to the robot root body mocap body
         * - Add a mocap body in the world body (called <RobotName>_Mocap)
         * - Add a weld constraint welding them together.
         */
        void setWithMocapBody(bool enabled);
        
        
    private:
        
        /// Set the output path members.
        void setPaths(const std::string& filename, const std::string& basePath, 
                      const std::string& meshRelDir);
        /// Create output directories if the do not exist.
        void ensureDirectoriesExist();
        
        /// Make the compiler section.
        void makeCompiler();
        /// Add a defaults group for the robot.
        void makeDefaultsGroup();
        /// Add a skybox texture asset.
        void addSkybox();
        
        /// Add a mocap body with defaults group.
        void addMocapBody();
        
        /// Construct the body structure corresponding to the robot nodes.
        void makeNodeBodies();
        /// Add a body element for a robot node. If its parent does not exist, 
        /// create the parent body first.
        mjcf::XMLElement* addNodeBody(RobotNodePtr node);
        
        /// Convert meshes and add mesh assets for robot nodes with visualization.
        void addNodeBodyMeshes();
        
        /// Add contact exclude elements for IgnoreCollision elements.
        void addContactExcludes();
        
        /// Add actuators for all joints.
        void addActuators();
        
        /// Scale all lengths by lengthScaling.
        void scaleLengths(XMLElement* elem);
        
        std::vector<const mjcf::XMLElement*> getAllElements(const std::string& elemName);

        
        // Paremeters
        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        /// The actuator type.
        ActuatorType actuatorType = ActuatorType::MOTOR;
        
        /// Add a mocap
        bool withMocapBody = false;
        
        
        // Paths
        
        boost::filesystem::path outputDirectory;
        boost::filesystem::path outputFileName;
        boost::filesystem::path outputMeshRelDirectory;
        boost::filesystem::path outputMeshDirectory() { return outputDirectory / outputMeshRelDirectory; }
        
        
        // Input
        
        /// The input robot.
        RobotPtr robot;
        
        
        // Output
        
        /// The built MJCF document.
        DocumentPtr document = nullptr;

        
        // Processing
        
        /// Sanitizes massless bodies.
        mjcf::MasslessBodySanitizer masslessBodySanitizer {document, robot};
        
        /// Map of robot node names to XML elements.
        std::map<std::string, mjcf::XMLElement*> nodeBodies;
        
        
        const std::string t = "| ";
        
    };
    
}
}
