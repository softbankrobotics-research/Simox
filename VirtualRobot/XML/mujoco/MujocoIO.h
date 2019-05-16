#pragma once

#include <filesystem>

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
        
        
        /// Set whether paths (e.g. meshes) shall be relative instead of absolute.
        /// This can cause problems when including the generated model from another directory.
        void setUseRelativePaths(bool useRelative);
        
        /// Set the scaling for lengths.
        void setLengthScale(float value);
        /// Set the scaling for meshes.
        void setMeshScale(float value);
        /// Set the scaling for mass (to kg).
        void setMassScale(float value);

        /// Set the actuator type.
        void setActuatorType(ActuatorType value);
        /// Set whether a type suffix shall be added to actuator names.
        void setAddActuatorTypeSuffix(bool enable);
        /// Set suffixes appended to actuator names if adding actuator type suffixes is enabled.
        void setActuatorTypeSuffixes(const std::map<ActuatorType, std::string>& suffixes);
        
        
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
        mjcf::Body addMocapBody();
        
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
        /// Add contact exclude elements for IgnoreCollision elements.
        void addMocapContactExcludes(mjcf::Body mocap);
        
        /// Add actuators for all joints.
        void addActuators();
        
        /// Scale all lengths by lengthScaling.
        void scaleLengths(mjcf::AnyElement elem);
        
        
        // Paremeters
        
        bool useRelativePaths = false;
        
        /// Scaling for lengths, such as positions and translations (to m).
        float lengthScale = 0.001f;
        /// Scaling for lengths, such as positions and translations (to m).
        float meshScale = 1.f;
        /// Scaling for mass (to kg).
        float massScale = 1.f;
        
        /// The actuator type.
        ActuatorType actuatorType = ActuatorType::MOTOR;
        
        /// Whether to add type suffix to actuator names
        bool addActuatorTypeSuffix = false;
        /// The suffixes added if addActuatorTypeSuffix is true.
        std::map<ActuatorType, std::string> actuatorTypeSuffixes = {
            { ActuatorType::MOTOR, "_motor" },
            { ActuatorType::POSITION, "_position" },
            { ActuatorType::VELOCITY, "_velocity" },
        };
        
        
        /// Add a mocap
        bool withMocapBody = false;
        
        /// Verbose printing.
        bool verbose = false;
        
        
        // Paths
        
        std::filesystem::path outputDirectory;
        std::filesystem::path outputFileName;
        std::filesystem::path outputMeshRelDirectory;
        std::filesystem::path outputMeshDirectory() { return outputDirectory / outputMeshRelDirectory; }
        
        
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
    
    
}}
