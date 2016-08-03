/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Adrian Knobloch
* @copyright  2016 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Model_h_
#define _VirtualRobot_Model_h_

#include "VirtualRobot.h"
#include "BoundingBox.h"
#include <ConditionedLock.h>
#include "Nodes/ModelNode.h"

#include <Eigen/Core>

namespace VirtualRobot
{
    /*!
     * This is the main object defining the kinematic structure of a model.
     *
     * \see RobotIO, RobotNode, RobotNodeSet, EndEffector
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Model
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         * Constructor.
         *
         * @param name Specifies the name of this instance.
         * @param type Specifies the type of the robot (e.g. multiple robots of the same type could exists with different names)
         */
        Model(const std::string& name, const std::string& type = "");

        /*!
         * Destructor.
         */
        virtual ~Model();

        /*!
         * This method is automatically called in ModelNode's constructor!
         *
         * Register a new node to this model
         *
         * @param node The new node.
         */
        virtual void registerModelNode(ModelNodePtr node);

        /*!
         * Deregister the given ModelNode.
         *
         * @param node The node to deregister.
         */
        virtual void deregisterModelNode(ModelNodePtr node);

        /*!
         * Check, if the ModelNode is registered to this model.
         *
         * @param node The node to check for.
         * @return True, if the node is registered; false otherwise.
         */
        virtual bool hasModelNode(ModelNodePtr node);

        /*!
         * Check, if the node is registered to this model.
         *
         * @param modelNodeName The name of the node to check for.
         * @return True, if the node is registered; false otherwise.
         */
        virtual bool hasModelNode(const std::string& modelNodeName);

        /*!
         * Get a pointer to the ModelNode, identified by the given name.
         *
         * @param modelNodeName The name of the ModelNode.
         * @return A pointer to the ModelNode.
         */
        virtual ModelNodePtr getModelNode(const std::string& modelNodeName);

        /*!
         * Get all nodes, registered to this model.
         *
         * @return The registered nodes.
         */
        virtual std::vector< ModelNodePtr > getModelNodes();

        /*!
         * Get all nodes, registered to this model.
         *
         * @param storeNodes The vector to store the nodes.
         * @param clearVector If true, the vector is cleared bevor storing nodes.
         */
        virtual void getModelNodes(std::vector< ModelNodePtr >& storeNodes, bool clearVector = true);

        /*!
         * Register a new ModelNodeSet to this model.
         *
         * @param nodeSet The new node set.
         */
        virtual void registerModelNodeSet(ModelNodeSetPtr nodeSet);

        /*!
         * Deregister the given ModelNodeSet.
         *
         * @param nodeSet The node set to deregister.
         */
        virtual void deregisterModelNodeSet(ModelNodeSetPtr nodeSet);

        /*!
         * Check, if the ModelNodeSet is registered to this model.
         *
         * @param nodeSet The node set to check for.
         * @return True, if the node set is registered; false otherwise.
         */
        virtual bool hasModelNodeSet(ModelNodeSetPtr nodeSet);

        /*!
         * Check, if the node set is registered to this model.
         *
         * @param nodeSet The name of the node set to check for.
         * @return True, if the node set is registered; false otherwise.
         */
        virtual bool hasModelNodeSet(const std::string& name);

        /*!
         * Get a pointer to the ModelNodeSet, identified by the given name.
         *
         * @param modelNodeName The name of the ModelNodeSet.
         * @return A pointer to the ModelNodeSet.
         */
        virtual ModelNodeSetPtr getModelNodeSet(const std::string& nodeSetName);

        /*!
         * Get all node sets, registered to this model.
         *
         * @return The registered node sets.
         */
        virtual std::vector<ModelNodeSetPtr> getModelNodeSets();

        /*!
         * Get all node sets, registered to this model.
         *
         * @param storeNodeSet The vector to store the node sets.
         * @param clearVector If true, the vector is cleared bevor storing nodes.
         */
        virtual void getModelNodeSets(std::vector<ModelNodeSetPtr>& storeNodeSet, bool clearVector = true);

        /*!
         * The root node is the first RobotNode of this robot.
         *
         * @param node The new root node.
         */
        virtual void setRootNode(ModelNodePtr node);

        /*!
         * Get the current root node.
         *
         * @return The current root node.
         */
        virtual ModelNodePtr getRootNode();

        /*!
         * Get the name of this model.
         *
         * @return The name.
         */
        virtual std::string getName();

        /*!
         * Get the type of this model.
         *
         * @return The type.
         */
        virtual std::string getType();

        /*!
         * Configures the model to threadsafe or not.
         * Per default the model is threadsafe, i.e., updating the
         * model state and reading the Poses from the nodes is mutual
         * exclusive. This feature can be turned off, however, in
         * order to be make data access faster in single threaded
         * applications.
         *
         * @param flag Set to true, if the model should be threadsafe and to false otherwise.
         */
        virtual void setThreadsafe(bool flag);

        /*!
         * Set the global position of this model.
         *
         * @param globalPose The new global pose.
         * @param applyValues If true, the global pose of all ModelNodes is adjusted.
         */
        virtual void setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyValues = true);

        /*!
         * Get the global Pose of this model.
         *
         * @return The global pose.
         */
        virtual Eigen::Matrix4f getGlobalPose();

        /*!
         * Set the global pose of this model so that the RobotNode node is at position globalPoseNode.
         *
         * @param node The node to set the position relative to.
         * @param globalPoseNode The global pose for the node.
         */
        virtual void setGlobalPoseForRobotNode(const RobotNodePtr& node, const Eigen::Matrix4f& globalPoseNode);

        /*!
         * Return center of mass of this robot in local coordinate frame.
         * All RobotNodes of this robot are considered according to their mass.
         *
         * @return The center of mass in local coordinate frame.
         */
        virtual Eigen::Vector3f getCoMLocal();

        /*!
         * Return Center of Mass of this robot in global coordinates.
         * All RobotNodes of this robot are considered according to their mass.
         *
         * @return The center of mass in global coordinate frame.
         */
        virtual Eigen::Vector3f getCoMGlobal();

        /*!
         * Get the scaling of this model.
         *
         * @return The scaling.
         */
        float getScaling();

        /*!
         * Set the scaling of this model.
         *
         * @param scaling The new scaling factor.
         */
        void setScaling(float scaling);

        /*!
         * Apply all joint values.
         */
        virtual void applyJointValues();

        /*!
            Shows the structure of the robot
        */
        void showStructure(bool enable, const std::string& type = "");

        /*!
            Shows the coordinate systems of the robot nodes
        */
        void showCoordinateSystems(bool enable, const std::string& type = "");

        /*!
         * Convenient method for highlighting the visualization of this robot.
         * It is automatically checked whether the collision model or the full model is part of the visualization.
         * @param visualization The visualization for which the highlighting should be performed.
         * @param enable On or off
         */
        virtual void highlight(VisualizationPtr visualization, bool enable);

        /*!
         * Display some physics debugging information.
         *
         * @param enableCoM If true, the center of mass is shown (if given). If a comModel is given it is used for visualization, otherwise a standrad marker is shown.
         * @param enableInertial If true, a visualization of the inertial matrix is shown (if given).
         * @param comModel If set, this visualization is used to display the CoM location. If not set, a standard marker is used.
         */
        void showPhysicsInformation(bool enableCoM, bool enableInertial, VisualizationNodePtr comModel = VisualizationNodePtr());

        /*!
         * Setup the full model visualization.
         *
         * @param showVisualization If false, the visualization is disabled.
         * @param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
         */
        void setupVisualization(bool showVisualization, bool showAttachedVisualizations);

        /*!
         * Enables/Disables the visualization updates of the visualization model.
         *
         * @param enable Set to true, to enable the updates and to false, to disable the updates.
         */
        void setUpdateVisualization(bool enable);

        /*!
         * Enables/Disables the visualization updates of the collision model.
         *
         * @param enable Set to true, to enable the updates and to false, to disable the updates.
         */
        void setUpdateCollisionModel(bool enable);

        /*!
         * Get the complete setup of all model nodes.
         *
         * @return The configuration of the model.
         */
        virtual RobotConfigPtr getConfig();

        /*!
         * Sets the configuration according to the ModelNodes, defined in c. All other nodes are not affected.
         *
         * @param c The new configuration.
         */
        virtual bool setConfig(RobotConfigPtr c);

        /*!
         * Set a joint value [rad].
         * The internal matrices and visualizations are updated accordingly.
         * If you intend to update multiple joints, use \ref setJointValues for faster access.
         *
         * @param rn The model node.
         * @param jointValue The new joint value.
         */
        virtual void setJointValue(ModelNodePtr rn, float jointValue);

        /*!
         * Set a joint value [rad].
         * The internal matrices and visualizations are updated accordingly.
         * If you intend to update multiple joints, use \ref setJointValues for faster access.
         *
         * @param nodeName The name of the node.
         * @param jointValue The new joint value.
         */
        virtual void setJointValue(const std::string& nodeName, float jointValue);

        /*!
         * Set joint values [rad].
         * The complete model is updated to apply the new joint values.
         *
         * @param jointValues A map containing ModelNode names with according values.
         */
        virtual void setJointValues(const std::map< std::string, float >& jointValues);

        /*!
         * Set joint values [rad].
         * The complete model is updated to apply the new joint values.
         *
         * @param jointValues A map containing ModelNodes with according values.
         */
        virtual void setJointValues(const std::map< ModelNodePtr, float >& jointValues);

        /*!
         * Set joint values [rad].
         * The complete model is updated to apply the new joint values.
         *
         * @param config The ModelConfig defines the ModelNodes and joint values.
         */
        virtual void setJointValues(RobotConfigPtr config);

        /*!
         * Set joint values [rad].
         * Only those joints in config are affected which are present in rns.
         * The subpart of the model, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
         *
         * @param rns Only joints in this rns are updated.
         * @param config The ModelConfig defines the ModelNodes and joint values.
         */
        virtual void setJointValues(ModelNodeSetPtr rns, RobotConfigPtr config); //TODO: to ModelNodeSet

        /*!
         * Apply configuration of trajectory at time t
         *
         * @param trajectory The trajectory
         * @param t The time (0<=t<=1)
         */
        virtual void setJointValues(TrajectoryPtr trajectory, float t);

        /*!
         * Get number of faces (i.e. triangles) of this object.
         *
         * @param collisionModel Indicates weather the faces of the collision model or the full model should be returned.
         */
        virtual int getNumFaces(bool collisionModel = false);

        /*!
         * The (current) bounding box in global coordinate system.
         *
         * @param collisionModel Either the collision or the visualization model is considered.
         * @return The bounding box.
         */
        virtual BoundingBox getBoundingBox(bool collisionModel = true);

        /*!
         * Get all collision models of this model.
         *
         * @return A vector containing all collision models.
         */
        virtual std::vector<CollisionModelPtr> getCollisionModels();

        /*!
         * Return accumulated mass of this robot.
         *
         * @return The mass.
         */
        virtual float getMass();

        /*!
         * Extract a sub kinematic from this robot and create a new robot instance.
         *
         * @param startJoint The kinematic starts with this RobotNode
         * @param newRobotType The name of the newly created robot type
         * @param newRobotName The name of the newly created robot
         * @param cloneRNS Clone all robot node sets that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
         * @param cloneEEFs Clone all end effectors that belong to the original robot and for which the remaining robot nodes of the subPart are sufficient.
         * @param collisionChecker The new robot can be registered to a different collision checker. If not set, the collision checker of the original robot is used.
         * @param scaling Can be set to create a scaled version of this robot. Scaling is applied on kinematic, visual, and collision data.
         */
        virtual RobotPtr extractSubPart(RobotNodePtr startJoint, const std::string& newRobotType, const std::string& newRobotName, bool cloneRNS = true, bool cloneEEFs = true, CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(), float scaling = 1.0f);

        /*!
         * Attach a new ModelNode to this model.
         *
         * @param newNode The node to attach.
         * @param existingNode The node to attach the new child at.
         */
        void attachChildTo(ModelNodePtr newNode, ModelNodePtr existingNode);

        /*!
         * Attach a new ModelNode to this model.
         *
         * @param newNode The node to attach.
         * @param existingNodeName The name of the node to attach the new child at.
         */
        void attachNodeTo(ModelNodePtr newNode, std::string& existingNodeName);

        /*!
         * Removes the node from this model.
         * This also removes all its children.
         *
         * @param node The node to remove.
         */
        void detachNode(ModelNodePtr node);

        /*!
         * Removes the node from this model.
         * This also removes all its children.
         *
         * @param nodeName The name of the node to remove.
         */
        void detachNode(std::string& nodeName);

        /*!
         * Just storing the filename.
         *
         * @param filename The filename to store.
         */
        virtual void setFilename(const std::string& filename);

        /*!
         * Retrieve the stored filename.
         *
         * @return The filename.
         */
        virtual std::string getFilename();

        /*!
         * Creates an XML string that defines the complete robot. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
         * \see RobotIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPath TODO: Documentation
         * @param storeEEF TODO: Documentation
         * @param storeRNS TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath = ".", const std::string& modelPath = "models", bool storeEEF = true, bool storeRNS = true, bool storeAttachments = true);

        /*!
         * Print status information.
         */
        virtual void print();

        /*!
         * This readlock can be used to protect data access. It locks the mutex until deletion.
         * For internal use. API users will usually not need this functionality since all data access is protected automatically.
         *
         * Exemplary usage:
         * {
         *     ReadLockPtr lock = robot->getReadLock();
         *     // now the mutex is locked
         *
         *     // access data
         *     // ...
         *
         * } // end of scope -> lock gets deleted and mutex is released automatically
         *
         * @return The lock object.
         */
        virtual ReadLockPtr getReadLock();

        /*!
         * This writelock can be used to protect data access. It locks the mutex until deletion.
         * For internal use. API users will usually not need this functionality since all data access is protected automatically.
         *
         * Exemplary usage:
         * {
         *     WriteLockPtr lock = robot->getWriteLock();
         *     // now the mutex is locked
         *
         *     // access data
         *     // ...
         *
         * } // end of scope -> lock gets deleted and mutex is released automatically
         *
         * @return The lock object.
         */
        virtual WriteLockPtr getWriteLock();

        /*!
         * Clones this robot.
         *
         * @param name The new name.
         * @param collisionChecker If set, the returned robot is registered with this col checker, otherwise the CollisionChecker of the original robot is used.
         * @param scaling Scale Can be set to create a scaled version of this robot. Scaling is applied on kinematic, visual, and collision data.
         */
        virtual RobotPtr clone(const std::string& name, CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(), float scaling = 1.0f);
    };
}

#endif // _VirtualRobot_Model_h_
