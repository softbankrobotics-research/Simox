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
#ifndef _VirtualRobot_ModelNode_h_
#define _VirtualRobot_ModelNode_h_

#include "../VirtualRobot.h"
#include "../Model.h"
#include "../ModelNodeSet.h"
#include "Attachments/ModelNodeAttachment.h"

#include <vector>
#include <map>
#include <string>
#include <cstdint>

#include <Eigen/Core>

#include <boost/enable_shared_from_this.hpp>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelNode : public boost::enable_shared_from_this<ModelNode>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         * The type of the node described as uint16_t.
         *
         * The last 8 bit can be used to determinate the main type of the node.
         * Currently there are the Types Link (0x0001) and Joint (0x0002).
         *
         * The first 8 bit can be used to determinate the sub type.
         *
         * Each sub/main type is only allowed to use one bit.
         */
        enum ModelNodeType : uint16_t
        {
            Node            = 0x0000,

            Link            = 0x0001,

            Joint           = 0X0002,
            JointFixed      = 0x0102,
            JointPrismatic  = 0x0202,
            JointRevolute   = 0x0402
        };

    protected:
        /*!
         * Constructor with settings.
         *
         * Only used by subclasses.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         */
        ModelNode(ModelWeakPtr model,
                  const std::string& name,
                  Eigen::Matrix4f& localTransformation);

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNode();

        /*!
         * Initialize ModelNode. Here pointers to parent and children are created from names.
         * Be sure all children are created and registered to the Model before calling initialize.
         * Usually RobotFactory manages the initialization.
         *
         * @param parent The parent of this node.
         * @param children The children of this node.
         */
        virtual void initialize(ModelNodePtr parent = ModelNodePtr(), const std::vector<ModelNodePtr>& children = std::vector<ModelNodePtr>());

        /*!
         * Get the @ref ModelNodeType of this Node.
         *
         * @return The type of this node.
         */
        virtual ModelNodeType getType() const = 0;

        /*!
         * Get the corresponding Model.
         *
         * @return A pointer to the model.
         */
        ModelPtr getModel() const;

        /*!
         * Get the name of this node.
         *
         * @return The name.
         */
        inline std::string getName() const;

        /*!
         * Check if this node is initialized.
         *
         * @return True, if this node is initialized; false otherwise.
         */
        inline bool isInitialized() const;

        /*!
         * Get the child with the given name.
         *
         * @param name The name of the child.
         * @return The child or a empty pointer, if this node does not have a child with the given name.
         */
        ModelNodePtr getChildByName(std::string& name) const;

        /*!
         * Get the parent Node.
         * If a type is given, the first parent of this type is returned.
         * If this node does not have a parent of the given type, this returns an empty pointer.
         *
         * Example:
         * A1
         * |
         * A2
         * |
         * B3
         * |
         * A4
         * A4.getParentNode(A) == A2
         *
         * @param type The type of the nodes to return.
         * @return The parent node.
         */
        ModelNodePtr getParentNode(ModelNodeType type = ModelNodeType::Node) const;

        /*!
         * Get all child Nodes.
         * If a type is given all nodes of the given type are returned, which are directly connected to this node,
         * or only separated by nodes of a other type.
         *
         * Example:
         * A1
         * | \
         * B2 A3
         * |  | \
         * B4 B5 B6
         * A1.getChildNodes(B) == {B2, B5, B6}
         *
         * @param type The type of the nodes to return.
         * @return The child Nodes.
         */
        std::vector<ModelNodePtr> getChildNodes(ModelNodeType type = ModelNodeType::Node) const;

        /*!
         * All children and their children (and so on) are collected.
         * If the type is given, only nodes of the given type are added.
         * The current instance is also added.
         *
         * @param storeNodes A initialized vector to store the collected nodes in.
         * @param type The type of the nodes to return.
         * @param clearVector If true, the store vector is cleared.
        */
        void collectAllNodes(std::vector<ModelNodePtr>& storeNodes,
                             ModelNodeType type = ModelNodeType::Node,
                             bool clearVector = true);

        /*!
         * Find all ModelNodes whose movements affect this ModelNode.
         * If the type is given, only nodes of the given type are returned.
         *
         * @param set If set, the search is limited to the rns.
         * @param type The type of the nodes to return.
         * @return The ModelNodes.
        */
        virtual std::vector<ModelNodePtr> getAllParents(ModelNodeSetPtr set = ModelNodeSetPtr(),
                                                        ModelNodeType type = ModelNodeType::Node);

        /*!
         * Check if node is a child from this node.
         *
         * @param node The node to check for.
         * @return True, if node is a child from this node; false otherwise.
         */
        bool hasChild(ModelNodePtr node) const;

        /*!
         * Check if this node has a child with the given name.
         *
         * @param nodeName The name of the node to check for.
         * @return True, if this node has a child with the given name; false otherwise.
         */
        bool hasChild(std::string nodeName) const;

        /*!
         * Attach a new Child to this node.
         *
         * @param newNode The new node.
         * @return True, if the node could be attached; false otherwise.
         */
        bool attachChild(ModelNodePtr newNode);

        /*!
         * Detach a node from this node.
         *
         * @param node The node to detach.
         * @return True, if the node was a child and could be detached; false otherwise.
         */
        bool detachChild(ModelNodePtr node);

        /*!
         * Detach a node from this node.
         *
         * @param nodeName The name of the node to detach.
         * @return True, if the node was a child and could be detached; false otherwise.
         */
        bool detachChild(std::string& nodeName);

        /*!
         * The preJoint/preVisualization transformation. This transformation is applied before the joint and the visualization.
         *
         * @return The local transformation.
         */
        virtual Eigen::Matrix4f getLocalTransformation() const;

        /*!
         * Set a new local transformation.
         *
         * @param localTransformation The new transformation.
         * @param updatePose If set to true, the pose of all children will be updated recursively.
         */
        virtual void setLocalTransformation(const Eigen::Matrix4f& localTransformation, bool updatePose = true);

        /*!
         * Get the globalPose of this node.
         *
         * This call locks the model's mutex.
         *
         * @return The global pose.
         */
        virtual Eigen::Matrix4f getGlobalPose() const;

        /*!
         * The pose of this node in the root coordinate system of the model.
         *
         * @return The pose in root frame.
         */
        virtual Eigen::Matrix4f getPoseInRootFrame() const;

        /*!
         * The position of this node in the root coordinate system of the model.
         *
         * @return The position in root frame.
        */
        virtual Eigen::Vector3f getPositionInRootFrame() const;

        /*!
         * Compute/Update the transformations of this joint and all child joints. Therefore the parent is queried for its pose.
         *
         * This method is called by the model in order to update the pose matrices.
         *
         * @param updateChildren If set to true, the children are updated recursively.
         * @param updateAttachments If set to true, the attachments are updated.
         */
        void updatePose(bool updateChildren = true, bool updateAttachments = true);

        /*!
         * Attach a new attachment to this node.
         * This method first checks, if the attachment is attachable.
         *
         * @param attachment The attachment to add.
         *
         * @return True, is the attachment is attachable; false otherwise.
         */
        bool attach(ModelNodeAttachmentPtr attachment);

        /*!
         * Check if a attachment is attached.
         *
         * @param attachment The attachment to check for.
         *
         * @return True, if the attachment is attached; false otherwise.
         */
        bool isAttached(ModelNodeAttachmentPtr attachment);

        /*!
         * Detach a attachment.
         *
         * @param attachment The attachment to remove.
         *
         * @return True, if the attachment was attached; false otherwise.
         */
        bool detach(ModelNodeAttachmentPtr attachment);

        /*!
         * Get all attachments with the given type.
         * If no type is given all attachments are returned.
         *
         * @param type The type of the attachments to get.
         * @return The attachments.
         */
        std::vector<ModelNodeAttachmentPtr> getAttachments(std::string type = "");

        /*!
         * Get all attachments with visualisation.
         *
         * @return The attachments.
         */
        std::vector<ModelNodeAttachmentPtr> getAttachmentsWithVisualisation() const;

        /*!
         * Print status information.
         *
         * @param printChildren If set to true, information about all children are printed.
         * @param printDecoration TODO: Documentation
         */
        virtual void print(bool printChildren = false, bool printDecoration = true) const;

        /*!
         * Creates an XML string that defines the ModelNode. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
         *
         * \see RobotIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPathRelative TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         *
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", bool storeAttachments = true);

        /*!
         * Clone this ModelNode.
         *
         * @param newModel The newly created ModelNode belongs to newModel.
         * @param cloneChildren If true, all children are cloned (and their children, etc).
         * @param initializeWithParent If given, the ModelNode is initialized with this parent.
         * @param colChecker Must only be set if the cloned ModelNode should be registered to a different collision checker instance.
         * @param scaling Scale Can be set to create a scaled version of this model. Scaling is applied on kinematic, visual, and collision data.
         *
         * @return The new ModelNode.
         */
        virtual ModelNodePtr clone(ModelPtr newModel, bool cloneChildren = true, RobotNodePtr initializeWithParent = RobotNodePtr(), CollisionCheckerPtr colChecker = CollisionCheckerPtr(), float scaling = 1.0f);

        /*!
         * Transforms the pose, given in global coordinate system, to the local coordinate system of this node.
         *
         * @param poseGlobal The pose, given in global coordinate system, that should be transformed to the local coordinate system of this node.
         * @return The transformed pose.
         */
        Eigen::Matrix4f toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const;

        /*!
         * Transforms a position, given in global coordinate system, to the local coordinate system of this node.
         *
         * @param positionGlobal The position, given in global coordinate system, that should be transformed to the local coordinate system of this node.
         * @return The transformed position.
         */
        Eigen::Vector3f toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const;

        /*!
         * Transforms the pose, given in local coordinate system, to the global coordinate system.
         *
         * @param poseLocal The pose, given in local coordinate system of this node, that should be transformed to the global coordinate system.
         * @return The transformed pose.
         */
        Eigen::Matrix4f toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const;

        /*!
         * Transforms the position, given in local coordinate system, to the global coordinate system.
         *
         * @param positionLocal The position, given in local coordinate system of this node, that should be transformed to the global coordinate system.
         * @return The transformed position.
         */
        Eigen::Vector3f toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const;

        /*!
         * Returns the transformation matrix from this object to otherObject.
         *
         * @param otherObject The object to transform to.
         * @return The transform matrix.
         */
        Eigen::Matrix4f getTransformationTo(const ModelNodePtr otherObject);

        /*!
         * Returns the transformation matrix from otherObject to this object.
         *
         * @param otherObject The object to transform from.
         * @return The transform matrix.
         */
        Eigen::Matrix4f getTransformationFrom(const ModelNodePtr otherObject);

    protected:
        virtual void updatePoseInternally(bool updateChildren, bool updateAttachments);
        inline bool checkNodeOfType(ModelNodePtr node, ModelNodeType type) const;

    private:
        bool initialized;
        ModelWeakPtr model;
        std::string name;

        ModelNodeWeakPtr parent;
        std::vector<ModelNodePtr> children;

        Eigen::Matrix4f localTransformation;
        Eigen::Matrix4f globalPose;

        std::map<std::string, std::vector<ModelNodeAttachmentPtr>> attachments;
        std::vector<ModelNodeAttachmentPtr> attachmentsWithVisualisation;
    };
}

#endif // _VirtualRobot_ModelNode_h_
