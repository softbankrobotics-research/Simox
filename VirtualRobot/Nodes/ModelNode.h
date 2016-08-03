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

#include <vector>
#include <string>
#include <cstdint>

#include <Eigen/Core>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         * The type of the node described as uint8_t.
         *
         * The last 4 bit can be used to determinate the main type of the node.
         * Currently there are the Types Link (0x01) and Joint (0X02).
         */
        enum ModelNodeType : uint8_t
        {
            Link            = 0x01,

            Joint           = 0X02,
            JointFixed      = 0x12,
            JointPrismatic  = 0x22,
            JointRevolute   = 0x32
        };

    protected:
        /*!
         * Constructor with settings.
         *
         * Only used by subclasses.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param type The type of this Node.
         * @param localTransformation The transformation from the parent of this node to this node.
         */
        ModelNode(ModelWeakPtr model,
                  const std::string& name,
                  ModelNodeType type,
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
         */
        virtual void initialize(ModelNodePtr parent = ModelNodePtr(), const std::vector<ModelNodePtr>& children = std::vector<ModelNodePtr>());

        /*!
         * Get the @ref ModelNodeType of this Node.
         *
         * @return The type of this node.
         */
        virtual ModelNodeType getType() = 0;

        /*!
         * Get the corresponding Model.
         *
         * @return A pointer to the model.
         */
        ModelPtr getModel() const;

        /*!
         * Get the parent Node.
         *
         * @return The parent node.
         */
        ModelNodePtr getParentNode() const;

        /*!
         * Get the first parent, which is a link.
         *
         * @return The parent node.
         */
        ModelNodePtr getParentLink() const;

        /*!
         * Get the first parent, which is a joint.
         *
         * @return The parent node.
         */
        ModelNodePtr getParentJoint() const;

        /*!
         * Get all child Nodes.
         *
         * @return The child Nodes.
         */
        std::vector<ModelNodePtr> getChildNodes() const;

        /*!
         * Get all joint nodes, which are directly connected to this node, or only separated by link nodes.
         *
         * @return The child Nodes.
         */
        std::vector<ModelNodePtr> getChildJoints() const;

        /*!
         * Get all link nodes, which are directly connected to this node, or only separated by joint nodes.
         *
         * @return The child Nodes.
         */
        std::vector<ModelNodePtr> getChildLinks() const;

        /*!
         * All children and their children (and so on) are collected.
         * The current instance is also added.
         *
         * @param storeNodes A initialized and empty vector to store the collected nodes in.
        */
        void collectAllNodes(std::vector<ModelNodePtr>& storeNodes) const;

        /*!
         * Find all ModelNodes whose movements affect this ModelNode.
         *
         * @param rns If set, the search is limited to the rns.
         *
         * @return The ModelNodes.
        */
        virtual std::vector<RobotNodePtr> getAllParents(RobotNodeSetPtr rns = RobotNodeSetPtr());

        /*!
         * The preJoint/preVisualization transformation. This transformation is applied before the joint and the visualization.
         *
         * @return The local transformation.
         */
        virtual Eigen::Matrix4f getLocalTransformation()
        {
            return localTransformation;
        }

        /*!
         * Set a new local transformation.
         *
         * @param localTransformation The new transformation.
         * @param updatePose If set to true, the pose of all children will be updated recursively.
         */
        virtual void setLocalTransformation(const Eigen::Matrix4f& localTransformation, bool updatePose = true)
        {
            ModelNode::localTransformation = localTransformation;
            if(updatePose)
            {
                ModelNode::updatePose(true);
            }
        }

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
         */
        virtual void updatePose(bool updateChildren = true);

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
        /*!
         * Queries parent for global pose and updates visualization accordingly
         */
        virtual void updateTransformationMatrices();

    private:
        bool initialized;
        ModelWeakPtr model;
        std::string name;

        ModelNodeWeakPtr parent;
        std::vector<ModelNodePtr> children;

        Eigen::Matrix4f localTransformation;
    };
}

#endif // _VirtualRobot_ModelNode_h_
