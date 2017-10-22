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

#include "../VirtualRobot.h"
#include "Nodes/ModelNode.h"
#include "Nodes/ModelLink.h"
#include "../Tools/BoundingBox.h"
#include "Frame.h"
#include "../Tools/ConditionedLock.h"
#include <Eigen/Core>
#include <map>

namespace VirtualRobot
{
    /*!
     * This is the main object defining the kinematic structure of a model.
     *
     * \see ModelIO, ModelNode, ModelNodeSet
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Model : public std::enable_shared_from_this<Model>, public Frame
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

        /*!
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
		virtual bool hasLink(const ModelLinkPtr& link) const;
		virtual bool hasJoint(const ModelJointPtr& joint) const;

        /*!
         * Check, if the node is registered to this model.
         *
         * @param modelNodeName The name of the node to check for.
         * @return True, if the node is registered; false otherwise.
         */
        virtual bool hasModelNode(const std::string& modelNodeName) const;
        virtual bool hasJoint(const std::string& jointName) const;
        virtual bool hasLink(const std::string& linkName) const;

        /*!
         * Get a pointer to the ModelNode, identified by the given name. This node could be a link or joint.
         *
         * @param modelNodeName The name of the ModelNode.
         * @return A pointer to the ModelNode.
         *
         *@see getLink
         *@see getJoint
         */
        virtual ModelNodePtr getModelNode(const std::string& modelNodeName) const;
        virtual ModelLinkPtr getLink(const std::string& modelNodeName) const;
        virtual ModelJointPtr getJoint(const std::string& modelNodeName) const;

        /*!
        * Check, if the frame is registered to this model (either a ModelNode or an attached entity).
        *
        * @param coord The frame to check for.
        * @return True, if the frame is registered; false otherwise.
        */
        virtual bool hasFrame(const FramePtr& coord) const;

        /*!
        * Check, if the frame is registered to this model.
        *
        * @param frameName The name of the frame to check for.
        * @return True, if the frame is registered; false otherwise.
        */
        virtual bool hasFrame(const std::string& frameName) const;

        /*!
        * Get a pointer to the Frame, identified by the given name.
        *
        * @param frameName The name of the Frame.
        * @return A pointer to the Frame.
        */
        virtual FramePtr getFrame(const std::string& frameName) const;


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
        virtual std::vector< ModelNodePtr > getModelNodes(const std::vector< std::string > & nodeNames) const;

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
        virtual std::vector<ModelJointPtr> getJoints() const;
        /*!
        * \brief getJoints Returns all joints of this model encapsulated in a JointSet
        * \return
        */
        virtual JointSetPtr getJointSet();

        /*!
         * \brief getLinks Returns all links of this model
         * \return
         */
        virtual std::vector<ModelLinkPtr> getLinks() const;

        /*!
        * \brief getLinks Returns all links of this model encapsulated in a LinkSet
        * \return
        */
        virtual LinkSetPtr getLinkSet();

        /*!
         * Get all node sets, registered to this model.
         *
         * @return The registered node sets.
         */
        virtual std::vector<ModelNodeSetPtr> getModelNodeSets() const;
        virtual std::vector<LinkSetPtr> getLinkSets() const;
        virtual std::vector<JointSetPtr> getJointSets() const;

        /*!
            Registers the ModelConfig to this robot. If a config  with the same name is already registered nothing happens.
        */
        virtual void registerConfiguration(const ModelConfigPtr &config);
        virtual void registerConfiguration(const std::vector<ModelConfigPtr> &configs);

        /*!
            Removes the configuration. If the ModelConfig is not registered nothing happens.
        */
        virtual void deRegisterConfiguration(const ModelConfigPtr &config);
        virtual void deRegisterConfiguration(const std::string& name);

        virtual bool hasConfiguration(const ModelConfigPtr &config) const;
        virtual bool hasConfiguration(const std::string& name) const;

        virtual ModelConfigPtr getConfiguration(const std::string& name) const;

        virtual std::vector< ModelConfigPtr > getConfigurations() const;

        template<typename T>
        std::vector< std::shared_ptr<T> > getAttachments() const;

        /*!
         * The root node is the first ModelNode of this model.
         *
         * @param node The new root node.
         * @param updatePose Recalculate all internal poses of the kinematic structure. This should be true unless you do the recalculation later.
         */
        virtual void setRootNode(const ModelNodePtr& node, bool updatePose = true);

        /*!
         * Get the current root node.
         *
         * @return The current root node.
         */
		virtual ModelNodePtr getRootNode() const;
		virtual ModelLinkPtr getFirstLink() const;

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
         * Set the global pose of this model so that the ModelNode/Frame node is at position globalPoseNode.
         *
         * @param node The frame/node to set the position relative to.
         * @param globalPoseNode The global pose for the node.
         */
        virtual void setGlobalPoseForModelNode(const FramePtr& node, const Eigen::Matrix4f& globalPoseNode);

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
         * A convenience function that creates and attaches a ModelStructure to each joint.
         * Each attached ModelStructure inherits the name of its corresponding joint appended by "_structure".
         * @param visualizationType The name of the VisualizationFactory (@see VisualizationFactory::fromName()) to use.
         *                          If not given, the default visualization factory is used.
         */
        void attachStructure(std::string visualizationType = "");

        /*!
         * A convenience function to detach ModelStructures.
         * This function basically reverts calls to attachStructure()
         */
        void detachStructure();

        /*!
         * A convenience function that creates and attaches a ModelFrame to each joint.
         * Each attached ModelFrame inherits the name of its corresponding joint appended by "_frame".
         * @param visualizationType The name of the VisualizationFactory (@see VisualizationFactory::fromName()) to use.
         *                          If not given, the default visualization factory is used.
         */
        void attachFrames(std::string visualizationType = "");

        /*!
         * A convenience function to detach ModelFrames.
         * This function basically reverts calls to attachFrames()
         */
        void detachFrames();

        // TODO: move to Visualization factory
        /*!
         * Convenient method for highlighting the visualization of this model.
         * It is automatically checked whether the collision model or the full model is part of the visualization.
         *
         * @param visualization The visualization for which the highlighting should be performed.
         * @param enable On or off
         */
        //virtual void highlight(const VisualizationPtr& visualization, bool enable);

        // TODO: move to Visualization factory
        /*!
         * Display some physics debugging information.
         *
         * @param enableCoM If true, the center of mass is shown (if given).
         *                  If a comModel is given it is used for visualization, otherwise a standrad marker is shown.
         * @param enableInertial If true, a visualization of the inertial matrix is shown (if given).
         * @param comModel If set, this visualization is used to display the CoM location.
         *                 If not set, a standard marker is used.
         */
        //void showPhysicsInformation(bool enableCoM, bool enableInertial,
        //                            const VisualizationNodePtr& comModel = VisualizationNodePtr());

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
         * Get the complete setup of all model nodes (i.e. the current configuration of the model).
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

        // TODO change ModelNode to ModelJoint in param
        /*!
         * Set joint values [rad].
         * The complete model is updated to apply the new joint values.
         *
         * @param jointValues A map containing ModelNodes with according values.
         */
        virtual void setJointValues(const std::map< ModelJointPtr, float >& jointValues);

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
         * @param cloneEEF Clone all end effectors that belong to the original model and for which the remaining model nodes of the subPart are sufficient.
         * @param collisionChecker The new model can be registered to a different collision checker. If not set, the collision checker of the original model is used.
         * @param scaling Can be set to create a scaled version of this model. Scaling is applied on kinematic, visual, and collision data.
         */
        virtual ModelPtr extractSubPart(const ModelNodePtr& startNode, 
										const std::string& newModelType,
                                        const std::string& newModelName, 
										bool cloneRNS = true,
										bool cloneEEF = true,
										const CollisionCheckerPtr& collisionChecker = CollisionCheckerPtr(),
                                        float scaling = 1.0f) const;

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
         * \see ModelIO::saveXML.
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
        ModelPtr clone(const std::string& name,
                               CollisionCheckerPtr collisionChecker = CollisionCheckerPtr(),
                               float scaling = 1.0f) const;

		bool hasEndEffector(const EndEffectorPtr &eef) const;
		bool hasEndEffector(const std::string &name) const;
		void registerEndEffector(const EndEffectorPtr &eef);
        void deregisterEndEffector(const EndEffectorPtr &eef);

        EndEffectorPtr getEndEffector(const std::string &name) const;
        std::vector<EndEffectorPtr> getEndEffectors() const;

        /**
         * @param linkVisuType The type of link visualization (e.g. collision).
         * @param visualizationType The name of the VisualizationFactory (@see VisualizationFactory::fromName()) to use.
         *                          If not set, the default VisualizationFactory (@see VisualizationFactory::getGlobalVisualizationFactory()) will be used.
         * @return A visualization of this model's links.
         */
        VisualizationPtr getVisualization(VirtualRobot::ModelLink::VisualizationType linkVisuType = VirtualRobot::ModelLink::Full, std::string visualizationType = "");

    protected:

        virtual void _clone(ModelPtr newModel,
                    const ModelNodePtr& startNode,
                    const CollisionCheckerPtr& collisionChecker = CollisionCheckerPtr(),
                    bool cloneRNS = true,
                    bool cloneEEF = true,
                    float scaling = 1.0f) const;
        std::string type;
        float scaling;

        bool threadsafe;
		mutable std::recursive_mutex mutex;

        ModelNodePtr rootNode;

        CollisionCheckerPtr collisionChecker;

        std::map<std::string, ModelNodePtr> modelNodeMap;
        std::map<std::string, ModelNodeSetPtr> modelNodeSetMap;
        std::map<std::string, EndEffectorPtr> eefMap;
        std::vector< RobotConfigPtr > configs;

        std::string filename;

        VisualizationPtr visualization;
        ModelLink::VisualizationType visuType;
    };
}

#endif // _VirtualRobot_Model_h_
