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
#ifndef _VirtualRobot_ModelNodeSet_h_
#define _VirtualRobot_ModelNodeSet_h_

#include "../Model/Model.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelNodeSet
    {
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
                     const ModelNodePtr kinematicRoot = ModelNodePtr(),
                     const FramePtr tcp = FramePtr());

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNodeSet();

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
        static ModelNodeSetPtr createModelNodeSet(const ModelPtr& model,
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
        static ModelNodeSetPtr createModelNodeSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<ModelNodePtr>& modelNodes,
                                                  const ModelNodePtr kinematicRoot = ModelNodePtr(),
                                                  const FramePtr tcp = FramePtr(),
                                                  bool registerToModel = false);
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

        ModelNodePtr& operator[](int i)
        {
            return getNode(i);
        }

        /*!
         * Get the node at position i.
         *
         * @param i The position of the node to get.
         * @return The node.
         */
        ModelNodePtr& getNode(int i);

        /*!
         * Iterator starting at the first object of this set.
         *
         * @return The iterator.
         */
        std::vector<ModelNodePtr>::iterator begin();

        /*!
         * Iterator starting at the last object of this set.
         *
         * @return The iterator.
         */
        std::vector<ModelNodePtr>::iterator end();

        /*!
         * Check, if this set contains the given node.
         *
         * @param node The node to check for.
         * @return True, if the node is contained; false otherwise.
         */
        bool hasModelNode(const ModelNodePtr& node) const;

        /*!
         * Check, if this set contains the given node.
         *
         * @param nodeName The name of the node to check for.
         * @return True, if the node is contained; false otherwise.
         */
        bool hasModelNode(const std::string &nodeName) const;

        /*!
         * Get all nodes of this set.
         *
         * @return The nodes contained in this set.
         */
        const std::vector<ModelNodePtr> getModelNodes() const;

        /*!
         * Returns the topmost node of the robot's kinematic tree to be used for updating all members of this set.
         * This node is usually defined in the RobotNodeSet's XML definition.
         *
         * @return The kinematic root.
         */
        ModelNodePtr getKinematicRoot() const;

        /*!
         * Set a new kinematic root.
         *
         * @param modelNode The new kinematic root.
         */
        void setKinematicRoot(const RobotNodePtr & modelNode);

        /*!
         * Returns the TCP.
         *
         * @return The new tcp.
         */
        FramePtr getTCP() const;

        /*!
         * Print out some information.
         */
        void print() const;

        /*!
         * Get the size of this set.
         *
         * @return The number of associated model nodes.
         */
        virtual unsigned int getSize() const;

        std::vector< std::string > getNodeNames() const;

        /*!
         * Returns true, if nodes (only name strings are checked) are sufficient for building this rns.
         * A set of nodes is sufficient, if it contains atleast all nodes of this ModelNodeSet.
         *
         * @param  nodes The nodes to check.
         * @return True, if the nodes are sufficient; false otherwise.
         */
        bool nodesSufficient(const std::vector<ModelNodePtr>& nodes) const;

        /*!
         * Create a XML string to represent this ModelNodeSet.
         *
         * @param tabs The number of tabs to start each line with.
         * @return The generated XML string.
         */
        virtual std::string toXML(int tabs) const;

        std::vector<ModelJointPtr> getModelJoints() const;

        std::vector<ModelLinkPtr> getModelLinks() const;

		/*!
		* Clone this modelset and register it to the new robot
		*/
		virtual ModelNodeSetPtr clone(ModelPtr newModel);

    protected:
        static ModelNodePtr checkKinematicRoot(const std::string &name, ModelPtr model);
        static ModelNodePtr checkTcp(const std::string &name, ModelPtr model);

        std::string name;
        ModelWeakPtr weakModel;
        std::vector<ModelNodePtr> modelNodes;
        ModelNodePtr kinematicRoot;
        FramePtr tcp;
    };
}

#endif // _VirtualRobot_ModelNodeSet_h_
