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
#ifndef _VirtualRobot_LinkSet_h_
#define _VirtualRobot_LinkSet_h_

#include "../Model/Model.h"
#include "ModelNodeSet.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT LinkSet : public ModelNodeSet
    {
    protected:
        /*!
         * Initialize this set with a vector of ModelNodes.
         *
         * @param name The name of this LinkSet.
         * @param model The associated model.
         * @param modelNodes The model nodes to add to this LinkSet.
         * @param kinematicRoot    This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                         kinematicRoot does not have to be a node of this set.
         *                         If not given, the first entry of modelNodes will be set as the kinematic root.
         * @param tcp   The tcp.
         *              If not given, the last entry of modelNodes will be set as the tcp.
         */
        LinkSet(const std::string& name,
                     const ModelWeakPtr& model,
                     const std::vector<ModelNodePtr>& modelNodes,
                     const ModelNodePtr kinematicRoot = ModelNodePtr(),
                     const FramePtr tcp = FramePtr());

    public:
        /*!
         * Destructor.
         */
        virtual ~LinkSet();

        /*!
         * Create a new LinkSet.
         *
         * @param model The associated model.
         * @param name The name of the new LinkSet.
         * @param modelNodeNames The names of the links to add.
         * @param kinematicRootName The name of the kinematic root.
         *                          This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                          The kinematic root does not have to be a node of this set.
         *                          If no name provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcpName The name of the tcp.
         *                The tcp does not have to be a node of this set.
         *                If no name provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new LinkSet is registered to the model.
         * @return The newly created LinkSet.
         */
        static LinkSetPtr createLinkSet(const ModelPtr& model,
                                        const std::string& name,
                                        const std::vector<std::string>& modelNodeNames,
                                        const std::string& kinematicRootName = "",
                                        const std::string& tcpName = "",
                                        bool registerToModel = false);
        /*!
         * Create a new LinkSet.
         *
         * @param model The associated model.
         * @param name The name of the new LinkSet.
         * @param modelNodes The nodes to add to this set.
         * @param kinematicRoot This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                      The kinematic root does not have to be a node of this set.
         *                      If no kinematic root provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcp The tcp.
         *            The tcp does not have to be a node of this set.
         *            If no tcp provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new LinkSet is registered to the model.
         * @return The newly created LinkSet.
         */
        static LinkSetPtr createLinkSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<ModelNodePtr>& modelNodes,
                                                  const ModelNodePtr kinematicRoot = ModelNodePtr(),
                                                  const FramePtr tcp = FramePtr(),
                                                  bool registerToModel = false);
        static LinkSetPtr createLinkSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<ModelLinkPtr>& modelNodes,
                                                  const ModelNodePtr kinematicRoot = ModelNodePtr(),
                                                  const FramePtr tcp = FramePtr(),
                                                  bool registerToModel = false);

        /*!
         * Get all nodes of this set.
         *
         * @return The nodes contained in this set.
         */
        const std::vector<ModelLinkPtr> getLinks() const;

		ModelLinkPtr& operator[](int i)
		{
			return getNode(i);
		}

		/*!
		* Get the node at position i.
		*
		* @param i The position of the node to get.
		* @return The node.
		*/
		ModelLinkPtr& getNode(int i);
		ModelLinkPtr& getLink(int i);


        /*!
         * Print out some information.
         */
        void print() const;

        /*!
         * Get the collision models of all contained nodes.
         *
         * @return The collision models.
         */
        std::vector<CollisionModelPtr> getCollisionModels();

        CollisionCheckerPtr getCollisionChecker() const;

        /*!
         * Get number of faces (i.e. triangles) of this object.
         *
         * @param collisionModel Indicates weather the faces of the collision model or the full model should be returned.
         */
        virtual int getNumFaces(bool collisionModel = false);

        /*!
         * Return center of mass of this node set.
         *
         * @return The CoM in global coordinate system.
         */
        Eigen::Vector3f getCoM();

        /*!
         * Return accumulated mass of this node set.
         *
         * @return The mass.
         */
        float getMass();

        /*!
         * Create a XML string to represent this LinkSet.
         *
         * @param tabs The number of tabs to start each line with.
         * @return The generated XML string.
         */
        virtual std::string toXML(int tabs) const;

		/*!
		* Clone this JointSet and register it to the new robot
		*/
		virtual ModelNodeSetPtr clone(ModelPtr newModel);


    protected:
        std::vector<ModelLinkPtr> links;
    };
}

#endif // _VirtualRobot_LinkSet_h_
