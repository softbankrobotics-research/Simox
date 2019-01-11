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

#include "../Model/Model.h"

#include <vector>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelNodeSet
    {
    private:
        class Implementation;
    public:
        /*!
         * Create a new ModelNodeSet.
         *
         * @param model The associated model.
         * @param name The name of the new ModelNodeSet.
         * @param modelNodeNames The names of the model nodes to add.
         * @param kinematicRootName The name of the kinematic root.
         *                          This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                          The kinematic root does not have to be a node of this set.
         *                          If no name provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcpName The name of the tcp.
         *                The tcp does not have to be a node of this set.
         *                If no name provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new ModelNodeSet is registered to the model.
         * @return The newly created ModelNodeSet.
         */
        static ModelNodeSetPtr createNodeSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<std::string>& modelNodeNames,
                                                  const std::string& kinematicRootName = "",
                                                  const std::string& tcpName = "",
                                                  bool registerToModel = false);

        /*!
         * Create a new ModelNodeSet.
         *
         * @param model The associated model.
         * @param name The name of the new ModelNodeSet.
         * @param modelNodes The nodes to add to this set.
         * @param kinematicRoot This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                      The kinematic root does not have to be a node of this set.
         *                      If no kinematic root provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcp The tcp.
         *            The tcp does not have to be a node of this set.
         *            If no tcp provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new ModelNodeSet is registered to the model.
         * @return The newly created ModelNodeSet.
         */
        static ModelNodeSetPtr createNodeSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<ModelNodePtr>& modelNodes,
                                                  const ModelNodePtr& kinematicRoot = ModelNodePtr(),
                                                  const FramePtr& tcp = FramePtr(),
                                                  bool registerToModel = false);

    protected:
        /*!
         * Initialize this set with a vector of ModelNodes.
         *
         * @param name The name of this ModelNodeSet.
         * @param model The associated model.
         * @param modelNodes The model nodes to add to this ModelNodeSet.
         * @param kinematicRoot    This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                         kinematicRoot does not have to be a node of this set.
         *                         If not given, the first entry of modelNodes will be set as the kinematic root.
         * @param tcp   The tcp.
         *              If not given, the last entry of modelNodes will be set as the tcp.
         */
        ModelNodeSet(const std::string& name,
                     const ModelWeakPtr& model,
                     const std::vector<ModelNodePtr>& modelNodes,
                     const ModelNodePtr& kinematicRoot = ModelNodePtr(),
                     const FramePtr& tcp = FramePtr());

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNodeSet();

        /*!
         * Get the name of this ModelNodeSet.
         *
         * @return The name.
         */
        std::string getName() const;

        /*!
         * Get the associated model.
         *
         * @return The model.
         */
        ModelPtr getModel() const;
        inline RobotPtr getRobot() const
        {
            return getModel();
        }

        /*!
         * Get the node at position i.
         *
         * @param i The position of the node to get.
         * @return The node.
         */
        virtual ModelNodePtr getNode(size_t i) const = 0;
        inline ModelNodePtr operator[](size_t i) const
        {
            return getNode(i);
        }

        /*!
         * Check, if this set contains the given node.
         *
         * @param node The node to check for.
         * @return True, if the node is contained; false otherwise.
         */
        virtual bool hasNode(const ModelNodePtr& node) const = 0;

        /*!
         * Check, if this set contains the given node.
         *
         * @param nodeName The name of the node to check for.
         * @return True, if the node is contained; false otherwise.
         */
        virtual bool hasNode(const std::string &nodeName) const = 0;

        /*!
         * Get all nodes of this set.
         *
         * @return The nodes contained in this set.
         */
        virtual std::vector<ModelNodePtr> getNodes() const = 0;
        virtual std::vector<ModelJointPtr> getJoints() const = 0;
        virtual std::vector<ModelLinkPtr> getLinks() const = 0;
        std::vector<std::string> getNodeNames() const;

        /*!
         * Returns the topmost node of the robot's kinematic tree to be used for updating all members of this set.
         * This node is usually defined in the RobotNodeSet's XML definition.
         *
         * @return The kinematic root.
         */
        virtual ModelNodePtr getKinematicRoot() const = 0;

        /*!
         * Set a new kinematic root.
         *
         * @param modelNode The new kinematic root.
         */
        virtual void setKinematicRoot(const ModelNodePtr & modelNode) = 0;

        /*!
         * Returns the TCP.
         *
         * @return The new tcp.
         */
        virtual FramePtr getTCP() const = 0;

        /*!
         * Print out some information.
         */
        virtual void print() const = 0;

        /*!
         * Get the size of this set.
         *
         * @return The number of associated model nodes.
         */
        virtual unsigned int getSize() const = 0;

        /*!
         * Returns true, if nodes (only name strings are checked) are sufficient for building this rns.
         * A set of nodes is sufficient, if it contains atleast all nodes of this ModelNodeSet.
         *
         * @param  nodes The nodes to check.
         * @return True, if the nodes are sufficient; false otherwise.
         */
        virtual bool nodesSufficient(const std::vector<ModelNodePtr>& nodes) const;

        /*!
         * Create a XML string to represent this ModelNodeSet.
         *
         * @param tabs The number of tabs to start each line with.
         * @return The generated XML string.
         */
        virtual std::string toXML(int tabs) const = 0;

		/*!
		 * Clones this ModelNodeSet and registers it to the given model.
		 * All nodes of this ModelNodeSet must be already registered at the given model.
		 * Note: this method does not deep copy the nodes, thus the cloned ModelNodeSet will point to the same nodes.
		 *
		 * @param model The model the cloned ModelNodeSet will be registered to.
         * @param newName The new Name of the set. If empty the old name will be used. (be aware, that only one set with the same name can be registered to a model)
         * @param registerToModel Override the default behaviour of registering the set to the model.
		*/
        virtual ModelNodeSetPtr clone(const ModelPtr& model, const std::string& newName = "", bool registerToModel = true) const = 0;

        virtual bool isKinematicChain() const;

        virtual bool isJointSet() const;
        virtual bool isLinkSet() const;

    protected:
        static ModelNodePtr checkKinematicRoot(const std::string &name, const ModelPtr& model);
        static FramePtr checkTcp(const std::string &tcpName, const ModelPtr& model);

        std::string name;
        ModelWeakPtr weakModel;
    };

    class ModelNodeSet::Implementation : public ModelNodeSet
    {
    public:
        Implementation(const std::string& name,
                     const ModelWeakPtr& model,
                     const std::vector<ModelNodePtr>& modelNodes,
                     const ModelNodePtr& kinematicRoot = ModelNodePtr(),
                     const FramePtr& tcp = FramePtr());

        virtual ModelNodePtr getNode(size_t i) const override;

        virtual bool hasNode(const ModelNodePtr &node) const override;
        virtual bool hasNode(const std::string &nodeName) const override;

        virtual std::vector<ModelNodePtr> getNodes() const override;
        virtual std::vector<ModelJointPtr> getJoints() const override;
        virtual std::vector<ModelLinkPtr> getLinks() const override;

        virtual unsigned int getSize() const override;

        virtual ModelNodePtr getKinematicRoot() const override;
        virtual void setKinematicRoot(const ModelNodePtr &modelNode) override;

        virtual FramePtr getTCP() const override;

        virtual void print() const override;
        virtual std::string toXML(int tabs) const override;

        virtual ModelNodeSetPtr clone(const ModelPtr& model, const std::string& newName = "", bool registerToModel = true) const override;

    protected:
        std::vector<ModelNodePtr> modelNodes;
        ModelNodePtr kinematicRoot;
        FramePtr tcp;
    };
}
