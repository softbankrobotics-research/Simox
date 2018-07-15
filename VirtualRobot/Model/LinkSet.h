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
    public:
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
                                        const std::vector<std::string>& linkNames,
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
                     const std::vector<ModelLinkPtr>& linkNodes,
                     const ModelNodePtr &kinematicRoot = ModelLinkPtr(),
                     const FramePtr& tcp = FramePtr());

    public:
        /*!
         * Destructor.
         */
        virtual ~LinkSet();

        virtual ModelNodePtr getNode(size_t i) const override;
        ModelLinkPtr getLink(size_t i) const;

        virtual bool hasNode(const ModelNodePtr &node) const override;
        virtual bool hasNode(const std::string &nodeName) const override;
        inline bool hasLink(const ModelLinkPtr &link) const
        {
            return hasNode(link);
        }
        inline bool hasLink(const std::string &linkName) const
        {
            return hasNode(linkName);
        }

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

        std::vector<CollisionModelPtr> getCollisionModels() const;
        CollisionCheckerPtr getCollisionChecker() const;
        int getNumFaces(bool collisionModel);
        Eigen::Vector3f getCoM();
        float getMass();

    protected:
        std::vector<ModelLinkPtr> links;
        ModelNodePtr kinematicRoot;
        FramePtr tcp;
    };
}

#endif // _VirtualRobot_LinkSet_h_
