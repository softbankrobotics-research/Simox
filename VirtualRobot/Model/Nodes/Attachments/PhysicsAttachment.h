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
* @author     Harry
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef SIMOX_PHYSICSATTACHMENT_H
#define SIMOX_PHYSICSATTACHMENT_H

#include "ModelNodeAttachment.h"

namespace VirtualRobot
{
    /**
     * An attachment visualizing the CoM and/or the inertia tensor when attached to a ModelLink.
     */
    class PhysicsAttachment : public ModelNodeAttachment
    {
        friend class ModelNode;
        friend class PhysicsAttachmentFactory;

    public:
        PhysicsAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation, const std::string &visualizationType);

        void enableVisualization(bool CoM, bool inertia);

        /**
         * Attaches this attachment to the given node.
         * @param node the node.
         * @return true, if the given node is a ModelLink, false otherwise.
         */
        bool isAttachable(const ModelNodePtr &node) override;

    protected:
        void setParent(const ModelNodePtr &node) override;

    private:
        void initVisualization();
    };

    typedef std::shared_ptr<PhysicsAttachment> PhysicsAttachmentPtr;
}

#endif //SIMOX_PHYSICSATTACHMENT_H
