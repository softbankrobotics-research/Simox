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
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelFrameFactory_h_
#define _VirtualRobot_ModelFrameFactory_h_

#include "ModelNodeAttachmentFactory.h"
#include "ModelFrame.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelFrameFactory : public ModelNodeAttachmentFactory
    {
    protected:
        /*!
         * Constructor.
         */
        ModelFrameFactory();

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelFrameFactory();


        /*!
         * Create a new attachment.
         *
         * @return The new attachment.
         */
        virtual ModelNodeAttachmentPtr createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform = Eigen::Matrix4f::Identity(), VisualizationNodePtr visu = VisualizationNodePtr());

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static std::shared_ptr<ModelNodeAttachmentFactory> createInstance(void*);

    private:
        static SubClassRegistry registry;
    };
    typedef std::shared_ptr<ModelFrameFactory> ModelFrameFactoryPtr;
}


#endif
