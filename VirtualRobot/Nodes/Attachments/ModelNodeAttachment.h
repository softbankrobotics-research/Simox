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
#ifndef _VirtualRobot_ModelNodeAttachment_h_
#define _VirtualRobot_ModelNodeAttachment_h_

#include "../../VirtualRobot.h"

#include <cstdint>

namespace VirtualRobot
{
    class ModelNodeAttachment
    {
    protected:
        /*!
         * Constructor.
         */
        ModelNodeAttachment();

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNodeAttachment();

        /*!
         * Checks if this attachment is attachable to the given node.
         * Mostly determined on the basis of the node type.
         *
         * @param node The node to check, if this attachment is attachable.
         *
         * @return True, if this attachment is attachable; false otherwise.
         */
        virtual bool isAttachable(ModelNodePtr node) = 0;

        /*!
         * Update the values of the Attachment.
         * This method is called by the node to inform the attachment about a change.
         */
        virtual void update() = 0;

        /*!
         * Get the visualisation of this attachment.
         * If this attachment does not have a visualisation, a null pointer is returned.
         *
         * @return The visualisation.
         */
        virtual VisualizationNodePtr getVisualisation()
        {
            return VisualizationNodePtr();
        }

        /*!
         * Get the type of this attachment.
         * This is used to seperate different attached attachments.
         *
         * @return The type of this attachment.
         */
        virtual std::string getType() = 0;
    };
}


#endif // _VirtualRobot_ModelNodeAttachment_h_
