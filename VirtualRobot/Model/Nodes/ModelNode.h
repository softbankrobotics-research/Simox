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
#pragma once

#include "../../VirtualRobot.h"
#include "../Frame.h"

#include <vector>
#include <map>
#include <string>
#include <cstdint>

#include <Eigen/Core>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelNode : public std::enable_shared_from_this<ModelNode>, public Frame
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
        enum NodeType : uint16_t
        {
            Node            = 0x0000,

            Link            = 0x0001,

            Joint           = 0X0002,
            JointFixed      = 0x0102,
            JointPrismatic  = 0x0202,
            JointRevolute   = 0x0402
        };

        /*!
         * Constructor with settings.
         *
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         */
        ModelNode(const ModelWeakPtr& model,
                  const std::string& name,
                  const Eigen::Matrix4f& localTransformation);

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNode() override;

        /*!
         * Initialize ModelNode. Here pointers to parent and children are created from names.
         * Be sure all children are created and registered to the Model before calling initialize.
         * Usually ModelFactory manages the initialization.
         *
         * @param parent The parent of this node.
         * @param children The children of this node.
         */
        //virtual void initialize(const ModelNodePtr& parent = ModelNodePtr(), const std::vector<ModelNodePtr>& children = std::vector<ModelNodePtr>());

        /*!
         * Get the @ref ModelNodeType of this Node.
         *
         * @return The type of this node.
         */
        virtual NodeType getType() const = 0;

        /*!
         * Get the corresponding Model.
         *
         * @return A pointer to the model.
         */
        ModelPtr getModel() const;

        /*!
         * Check if this node is initialized.
         *
         * @return True, if this node is initialized; false otherwise.
         */
        //bool isInitialized() const;

        /*!
         * Get the child with the given name.
         *
         * @param name The name of the child.
         * @return The child or a empty pointer, if this node does not have a child with the given name.
         */
        ModelNodePtr getChildByName(const std::string& name) const;

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
        ModelNodePtr getParentNode(NodeType type = NodeType::Node) const;

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
        std::vector<ModelNodePtr> getChildNodes(NodeType type = NodeType::Node) const;

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
                             NodeType type = NodeType::Node,
                             bool clearVector = true);

        /*!
         * Find all ModelNodes whose movements affect this ModelNode.
         * If the type is given, only nodes of the given type are returned.
         *
         * @param set If set, the search is limited to the rns.
         * @param type The type of the nodes to return.
         * @return The ModelNodes.
        */
        virtual std::vector<ModelNodePtr> getAllParents(const ModelNodeSetPtr& set = ModelNodeSetPtr(),
                                                        NodeType type = NodeType::Node) const;

        /*!
         * Check if node is a child from this node.
         *
         * @param node The node to check for.
         * @return True, if node is a child from this node; false otherwise.
         */
        virtual bool hasChild(const ModelNodePtr& node, bool recursive = false) const;

        /*!
         * Check if this node has a child with the given name.
         *
         * @param nodeName The name of the node to check for.
         * @return True, if this node has a child with the given name; false otherwise.
         */
        virtual bool hasChild(const std::string& nodeName, bool recursive = false) const;

        /*!
         * Attach a new Child to this node.
         *
         * @param newNode The new node.
         * @param updatePoses Indicates if the internal poses should be recalculated (should be true, unless recalculation takes place later)
         * @return True, if the node could be attached; false otherwise.
         */
        bool attachChild(const ModelNodePtr& newNode, bool updatePoses = true);

        /*!
         * Detach a node from this node.
         *
         * @param node The node to detach.
         * @return True, if the node was a child and could be detached; false otherwise.
         */
        bool detachChild(const ModelNodePtr& node);

        /*!
         * Detach a node from this node.
         *
         * @param nodeName The name of the node to detach.
         * @return True, if the node was a child and could be detached; false otherwise.
         */
        bool detachChild(const std::string& nodeName);

        /*!
         * The preJoint/preVisualization transformation. This transformation is applied before the joint and the visualization.
         *
         * @return The local transformation.
         */
        virtual Eigen::Matrix4f getLocalTransformation() const;

        /*!
         * Get the transformation performed by this node.
         * This includes the joint transformation.
         *
         * @return The transformation.
         */
        virtual Eigen::Matrix4f getNodeTransformation() const;

        /*!
         * Set a new preJoint/preVisualization transformation.
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
        virtual Eigen::Matrix4f getGlobalPose() const override;

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
        bool attach(const ModelNodeAttachmentPtr& attachment);

        /*!
         * Check if an attachment is attached.
         *
         * @param attachment The attachment to check for.
         *
         * @return True, if the attachment is attached; false otherwise.
         */
        bool isAttached(const ModelNodeAttachmentPtr& attachment);

        /*!
         * Detach an attachment.
         *
         * @param attachment The attachment to remove.
         *
         * @return True, if the attachment was attached before; false otherwise.
         */
        bool detach(const ModelNodeAttachmentPtr& attachment);

        /*!
         * Detach an attachment.
         *
         * \param attachmentName The name of the attachment to remove.
         * \return True, if the attachment was attached before; false otherwise.
         */
        bool detach(const std::string& attachmentName);

        /*!
         * Get all attachments with the given type.
         * If no type is given all attachments are returned.
         *
         * @param type The type of the attachments to get.
         * @return The attachments.
         */
        std::vector<ModelNodeAttachmentPtr> getAttachments(const std::string& type = "") const;

        template<typename T>
        std::vector< std::shared_ptr<T> > getAttachments() const
        {
            std::vector<std::shared_ptr<T> > result;
            for (const auto &a : attachments)
            {
                for (const auto &b : a.second)
                {
                    std::shared_ptr<T> n = std::dynamic_pointer_cast<T>(b);
                    if (n)
                        result.push_back(n);
                }
            }
            return result;
        }

        /*!
         * Get all attachments with visualisation.
         *
         * @return The attachments.
         */
        std::vector<ModelNodeAttachmentPtr> getAttachmentsWithVisualisation() const;

        bool hasAttachment(const std::string& attachmentName) const;
        ModelNodeAttachmentPtr getAttachment(const std::string& attachmentName) const;

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
         * \see ModelIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPathRelative TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         *
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", bool storeAttachments = true) = 0;

        /*!
         * Clones this ModelNode and registers it to the given model.
         *
         * @param newModel The model to which the cloned node will be registered to.
         * @param cloneChildren If true, all children will be cloned recursively.
         * @param cloneAttachments If true, attachments will be cloned also.
         * @param parentNode If given, the cloned node will be attached as a child to this node.
         * @param scaling Scale can be set to create a scaled version of this model. Scaling is applied on kinematic, visual, and collision data.
         *
         * @return The new ModelNode.
         */
        virtual ModelNodePtr clone(ModelPtr newModel, bool cloneChildren = true, bool cloneAttachments = true, ModelNodePtr parentNode = ModelNodePtr(), float scaling = 1.0f);

        /*!
         * Check if node has the given type.
         *
         * @param node The node to check.
         * @param type The type to check.
         * @return True, if the node has the given type; false otherwise.
         */
        inline static bool checkNodeOfType(const ModelNodePtr& node, NodeType type)
        {
            return (node->getType() & type) == type;
        }

        virtual bool isJoint()
        {
            return (getType() & NodeType::Joint)!=0;
        }
        virtual bool isTranslationalJoint()
        {
            return (getType() == NodeType::JointPrismatic);
        }
        virtual bool isRotationalJoint()
        {
            return (getType() == NodeType::JointRevolute);
        }
        virtual bool isLink()
        {
            return (getType() & NodeType::Link)!=0;
        }

    protected:
        ModelNode() = default;
        virtual void updatePoseInternally(bool updateChildren, bool updateAttachments);

        virtual ModelNodePtr _clone(ModelPtr newModel, float scaling = 1.0f) = 0;

        ModelWeakPtr model;

        ModelNodeWeakPtr parent;
        std::vector<ModelNodePtr> children;

        Eigen::Matrix4f localTransformation;

        std::map<std::string, std::vector<ModelNodeAttachmentPtr>> attachments;
    };
}
