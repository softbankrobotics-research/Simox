#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>

#include "MasslessBodySanitizer.h"


namespace VirtualRobot
{
namespace mujoco
{

    enum class ActuatorType
    {
        MOTOR, POSITION, VELOCITY
    };
    ActuatorType toActuatorType(const std::string& string);
    

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
        
        void setVerbose(bool value);
        
        
    private:
        
        /// Set the output path members.
        void setPaths(const std::string& filename, const std::string& basePath, 
                      const std::string& meshRelDir);
        /// Create output directories if the do not exist.
        void ensureDirectoriesExist();
        
        /// Make the compiler section.
        void makeCompiler();
        /// Add a defaults group for the robot.
        void makeDefaultsClass();
        /// Add a skybox texture asset.
        void addSkybox();
        
        /// Add a mocap body with defaults group.
        void addMocapBody();
        
        /// Construct the body structure corresponding to the robot nodes.
        void makeNodeBodies();
        
        /// Add a body element for a robot node.
        /// If its parent does not exist, create the parent body first.
        mjcf::Body addNodeBody(RobotNodePtr node);
        
        /// Add a body for the given node as child of parent and return it.
        mjcf::Body addNodeBody(mjcf::Body parent, RobotNodePtr node);
        /// Add a joint for the given node in body and return it.
        mjcf::Joint addNodeJoint(mjcf::Body body, RobotNodePtr node);
        /// Add an inertial for the given node in body and return it.
        mjcf::Inertial addNodeInertial(mjcf::Body body, RobotNodePtr node);
        
        /// Convert meshes and add mesh assets for robot nodes with visualization.
        void addNodeBodyMeshes();
        
        /// Add contact exclude elements for IgnoreCollision elements.
        void addContactExcludes();
        
        /// Add actuators for all joints.
        void addActuators();
        
        /// Scale all lengths by lengthScaling.
        void scaleLengths(mjcf::AnyElement elem);
        
        
        // Paremeters
        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        /// The actuator type.
        ActuatorType actuatorType = ActuatorType::MOTOR;
        
        /// Add a mocap
        bool withMocapBody = false;
        
        bool verbose = false;
        
        
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
        mjcf::DocumentPtr document = nullptr;
        mjcf::Body robotRoot;

        
        // Processing
        
        /// Sanitizes massless bodies.
        mujoco::MasslessBodySanitizer masslessBodySanitizer { robot };
        
        /// Map of robot node names to XML elements.
        std::map<std::string, mjcf::Body> nodeBodies;
        
        const std::string t = "| ";
        
    };
    
    
}
}
