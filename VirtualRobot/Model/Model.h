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

#include "../Model/Model.h"
#include "../Tools/BoundingBox.h"
#include "../Tools/ConditionedLock.h"
#include "Coordinate.h"
#include "Nodes/ModelNode.h"

#include <Eigen/Core>
#include <map>

namespace VirtualRobot
{
    /*!
     * This is the main object defining the kinematic structure of a model.
     *
     * \see RobotIO, ModelNode, ModelNodeSet
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Model : public std::enable_shared_from_this<Model>, Coordinate
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         * Constructor.
         *
         * @param name Specifies the name of this instance.
         * @param type Specifies the type of the model
         *             (e.g. multiple models of the same type could exists with different names)
         */
        Model(const std::string& name, const std::string& type = "");

        /*!
         * Destructor.
         */
        virtual ~Model();

		virtual void setName(const std::string &name);

        /*!
         * This method is automatically called in ModelNode's constructor!
         *
         * Register a new node to this model
         *
         * @param node The new node.
         */
        virtual void registerModelNode(const ModelNodePtr& node);

        /*!
         * Deregister the given ModelNode.
         *
         * @param node The node to deregister.
         */
        virtual void deregisterModelNode(const ModelNodePtr& node);

        /*!
         * Check, if the ModelNode is registered to this model.
         *
         * @param node The node to check for.
         * @return True, if the node is registered; false otherwise.
         */
        virtual bool hasModelNode(const ModelNodePtr& node) const;

        /*!
         * Check, if the node is registered to this model.
         *
         * @param modelNodeName The name of the node to check for.
         * @return True, if the node is registered; false otherwise.
         */
        virtual bool hasModelNode(const std::string& modelNodeName) const;

        /*!
         * Get a pointer to the ModelNode, identified by the given name.
         *
         * @param modelNodeName The name of the ModelNode.
         * @return A pointer to the ModelNode.
         */
        virtual ModelNodePtr getModelNode(const std::string& modelNodeName) const;

        /*!
        * Check, if the Coordinate is registered to this model (either a ModelNode or an attached entity).
        *
        * @param coord The coordinate to check for.
        * @return True, if the coordinate is registered; false otherwise.
        */
        virtual bool hasCoordinate(const CoordinatePtr& coord) const;

        /*!
        * Check, if the coordinate is registered to this model.
        *
        * @param coordinateName The name of the coordinate to check for.
        * @return True, if the coord is registered; false otherwise.
        */
        virtual bool hasCoordinate(const std::string& coordinateName) const;

        /*!
        * Get a pointer to the Coordinate, identified by the given name.
        *
        * @param coordinateName The name of the Coordinate.
        * @return A pointer to the Coordinate.
        */
        virtual CoordinatePtr getCoordinate(const std::string& coordinateName) const;


        /*!
         * Get all nodes, registered to this model.
         *
         * @param type If set, only nodes of this type are returned.
         * @return The registered nodes.
         */
        virtual std::vector< ModelNodePtr > getModelNodes(ModelNode::ModelNodeType type = ModelNode::ModelNodeType::Node) const;

        /*!
         * Get all nodes by names.
         *
         * @param type If set, only nodes of this type are returned.
         * @return The registered nodes.
         */
        virtual std::vector< ModelNodePtr > getModelNodes(std::vector< std::string > & nodeNames) const;

          /*!
         * Get all nodes, registered to this model.
         *
         * @param storeNodes The vector to store the nodes.
         * @param clearVector If true, the vector is cleared bevor storing nodes.
         * @param type If set, only nodes of this type are returned.
         */
        virtual void getModelNodes(std::vector< ModelNodePtr >& storeNodes, bool clearVector = true,
                                   ModelNode::ModelNodeType type = ModelNode::ModelNodeType::Node) const;

        /*!
         * Register a new ModelNodeSet to this model.
         *
         * @param nodeSet The new node set.
         */
        virtual void registerModelNodeSet(const ModelNodeSetPtr& nodeSet);
        virtual void registerJointSet(const JointSetPtr& nodeSet);
        virtual void registerLinkSet(const LinkSetPtr& nodeSet);

        /*!
         * Deregister the given ModelNodeSet.
         *
         * @param nodeSet The node set to deregister.
         */
        virtual void deregisterModelNodeSet(const ModelNodeSetPtr& nodeSet);
        virtual void deregisterJointSet(const JointSetPtr& nodeSet);
        virtual void deregisterLinkSet(const LinkSetPtr& nodeSet);

        /*!
         * Check, if the ModelNodeSet is registered to this model.
         *
         * @param nodeSet The node set to check for.
         * @return True, if the node set is registered; false otherwise.
         */
        virtual bool hasModelNodeSet(const ModelNodeSetPtr& nodeSet) const;
        virtual bool hasJointSet(const JointSetPtr& nodeSet) const;
        virtual bool hasLinkSet(const LinkSetPtr& nodeSet) const;

        /*!
         * Check, if the node set is registered to this model.
         *
         * @param nodeSet The name of the node set to check for.
         * @return True, if the node set is registered; false otherwise.
         */
        virtual bool hasModelNodeSet(const std::string& name) const;
        virtual bool hasJointSet(const std::string& name) const;
        virtual bool hasLinkSet(const std::string& name) const;

        /*!
         * Get a pointer to the ModelNodeSet, identified by the given name.
         *
         * @param modelNodeName The name of the ModelNodeSet.
         * @return A pointer to the ModelNodeSet.
         */
        virtual ModelNodeSetPtr getModelNodeSet(const std::string& nodeSetName) const;
        virtual JointSetPtr getJointSet(const std::string& nodeSetName) const;
        virtual LinkSetPtr getLinkSet(const std::string& nodeSetName) const;

        /*!
         * \brief getJoints Returns all joints of this model
         * \return
         */
        virtual JointSetPtr getJoints() const;

        /*!
         * \brief getLinks Returns all links of this model
         * \return
         */
        virtual LinkSetPtr getLinks() const;

        /*!
         * Get all node sets, registered to this model.
         *
         * @return The registered node sets.
         */
        virtual std::vector<ModelNodeSetPtr> getModelNodeSets() const;

        /*!
         * The root node is the first ModelNode of this model.
         *
         * @param node The new root node.
         */
        virtual void setRootNode(const ModelNodePtr& node);

        /*!
         * Get the current root node.
         *
         * @return The current root node.
         */
        virtual ModelNodePtr getRootNode() const;

        /*!
         * Get the name of this model.
         *
         * @return The name.
         */
        virtual std::string getName() const;

        /*!
         * Get the type of this model.
         *
         * @return The type.
         */
        virtual std::string getType() const;

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
        virtual Eigen::Matrix4f getGlobalPose() const;
        virtual Eigen::Vector3f getGlobalPosition() const;


        /*!
         * Set the global pose of this model so that the ModelNode node is at position globalPoseNode.
         *
         * @param node The node to set the position relative to.
         * @param globalPoseNode The global pose for the node.
         */
        virtual void setGlobalPoseForModelNode(const ModelNodePtr& node, const Eigen::Matrix4f& globalPoseNode);

        /*!
         * Return center of mass of this model in local coordinate frame.
         * All ModelNodes of this model are considered according to their mass.
         *
         * @return The center of mass in local coordinate frame.
         */
        virtual Eigen::Vector3f getCoMLocal() const;

        /*!
         * Return Center of Mass of this model in global coordinates.
         * All ModelNodes of this model are considered according to their mass.
         *
         * @return The center of mass in global coordinate frame.
         */
        virtual Eigen::Vector3f getCoMGlobal() const;

        /*!
         * Get the scaling of this model.
         *
         * @return The scaling.
         */
        float getScaling() const;

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
         * Shows the structure of the model.
         * This adds or removes a Attachment.
         *
         * @param enable If true, the structure is shown; if false it is removed.
         */
        void showStructure(bool enable);

        /*!
         * Shows the coordinate systems of the model nodes.
         * This adds or removes a Attachment.
         *
         * @param enable If true, the coordinate system is shown; if false it is removed.
         */
        void showCoordinateSystems(bool enable);

        /*!
         * Convenient method for highlighting the visualization of this model.
         * It is automatically checked whether the collision model or the full model is part of the visualization.
         *
         * @param visualization The visualization for which the highlighting should be performed.
         * @param enable On or off
         */
        virtual void highlight(const VisualizationPtr& visualization, bool enable);

        /*!
         * Display some physics debugging information.
         *
         * @param enableCoM If true, the center of mass is shown (if given).
         *                  If a comModel is given it is used for visualization, otherwise a standrad marker is shown.
         * @param enableInertial If true, a visualization of the inertial matrix is shown (if given).
         * @param comModel If set, this visualization is used to display the CoM location.
         *                 If not set, a standard marker is used.
         */
        void showPhysicsInformation(bool enableCoM, bool enableInertial,
                                    const VisualizationNodePtr& comModel = VisualizationNodePtr());

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
		bool getUpdateVisualization() const;

        /*!
         * Enables/Disables the visualization updates of the collision model.
         *
         * @param enable Set to true, to enable the updates and to false, to disable the updates.
         */
        void setUpdateCollisionModel(bool enable);
		bool getUpdateCollisionModel() const;

        /*!
         * Get the complete setup of all model nodes.
         *
         * @return The configuration of the model.
         */
        virtual ModelConfigPtr getConfig();

        /*!
         * Sets the configuration according to the ModelNodes, defined in c. All other nodes are not affected.
         *
         * @param c The new configuration.
         */
        virtual bool setConfig(const ModelConfigPtr& c);

        /*!
         * Set a joint value [rad].
         * The internal matrices and visualizations are updated accordingly.
         * If you intend to update multiple joints, use \ref setJointValues for faster access.
         *
         * @param node The model node.
         * @param jointValue The new joint value.
         */
        virtual void setJointValue(const ModelNodePtr& node, float jointValue);

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
        virtual void setJointValues(const ModelConfigPtr& config);

        /*!
         * Apply configuration of trajectory at time t
         *
         * @param trajectory The trajectory
         * @param t The time (0<=t<=1)
         */
        virtual void setJointValues(const TrajectoryPtr& trajectory, float t);

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
        virtual std::vector<CollisionModelPtr> getCollisionModels() const;

        /*!
         * Get the collision checker of this model.
         *
         * @return The collision checker.
         */
        virtual CollisionCheckerPtr getCollisionChecker() const;

        /*!
         * Return accumulated mass of this model.
         *
         * @return The mass.
         */
        virtual float getMass() const;

        /*!
         * Extract a sub kinematic from this model and create a new model instance.
         *
         * @param startNode The kinematic starts with this ModelNode
         * @param newModelType The name of the newly created model type
         * @param newModelName The name of the newly created model
         * @param cloneRNS Clone all model node sets that belong to the original model and for which the remaining model nodes of the subPart are sufficient.
         * @param cloneEEFs Clone all end effectors that belong to the original model and for which the remaining model nodes of the subPart are sufficient.
         * @param collisionChecker The new model can be registered to a different collision checker. If not set, the collision checker of the original model is used.
         * @param scaling Can be set to create a scaled version of this model. Scaling is applied on kinematic, visual, and collision data.
         */
        virtual ModelPtr extractSubPart(const ModelNodePtr& startNode, const std::string& newModelType,
                                        const std::string& newModelName, bool cloneRNS = true,
                                        const CollisionCheckerPtr& collisionChecker = CollisionCheckerPtr(),
                                        float scaling = 1.0f);

        /*!
         * Attach a new ModelNode to this model.
         * This registeres the node to this model.
         *
         * @param newNode The node to attach.
         * @param existingNode The node to attach the new child at.
         */
        void attachNodeTo(const ModelNodePtr& newNode, const ModelNodePtr& existingNode);

        /*!
         * Attach a new ModelNode to this model.
         * This registeres the node to this model.
         *
         * @param newNode The node to attach.
         * @param existingNodeName The name of the node to attach the new child at.
         */
        void attachNodeTo(const ModelNodePtr& newNode, const std::string& existingNodeName);

        /*!
         * Removes the node from this model.
         * This also removes all its children and deregisters the nodes.
         *
         * @param node The node to remove.
         */
        void detachNode(const ModelNodePtr& node);

        /*!
         * Removes the node from this model.
         * This also removes all its children and deregisters the nodes.
         *
         * @param nodeName The name of the node to remove.
         */
        void detachNode(const std::string& nodeName);

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
        virtual std::string getFilename() const;

        /*!
         * Creates an XML string that defines the complete model.
         * Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
         * \see RobotIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPath TODO: Documentation
         * @param storeRNS TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath = ".", const std::string& modelPath = "models",
                                  bool storeEEF = true, bool storeRNS = true, bool storeAttachments = true);

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
         *     ReadLockPtr lock = model->getReadLock();
         *     // now the mutex is locked
         *
         *     // access data
         *     // ...
         *
         * } // end of scope -> lock gets deleted and mutex is released automatically
         *
         * @return The lock object.
         */
        virtual ReadLockPtr getReadLock() const;

        /*!
         * This writelock can be used to protect data access. It locks the mutex until deletion.
         * For internal use. API users will usually not need this functionality since all data access is protected automatically.
         *
         * Exemplary usage:
         * {
         *     WriteLockPtr lock = model->getWriteLock();
         *     // now the mutex is locked
         *
         *     // access data
         *     // ...
         *
         * } // end of scope -> lock gets deleted and mutex is released automatically
         *
         * @return The lock object.
         */
        virtual WriteLockPtr getWriteLock() const;

        /*!
         * Clones this model.
         *
         * @param name The new name.
         * @param collisionChecker If set, the returned model is registered with this col checker, otherwise the CollisionChecker of the original model is used.
         * @param scaling Scale Can be set to create a scaled version of this model. Scaling is applied on kinematic, visual, and collision data.
         */
        virtual ModelPtr clone(const std::string& name, CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                               float scaling = 1.0f);

     
    private:
        std::string name;
        std::string type;
        float scaling;

        bool threadsafe;
        boost::recursive_mutex mutex;

        ModelNodePtr rootNode;

        CollisionCheckerPtr collisionChecker;

        std::map<std::string, ModelNodePtr> modelNodeMap;
        std::map<std::string, ModelNodeSetPtr> modelNodeSetMap;

        std::string filename;
    };
}

#endif // _VirtualRobot_Model_h_
