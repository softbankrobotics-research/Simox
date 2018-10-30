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
#pragma once

#include "ModelNodeAttachmentFactory.h"
#include "ModelStructure.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelStructureFactory : public ModelNodeAttachmentFactory
    {
    protected:
        /*!
         * Constructor.
         */
        ModelStructureFactory();

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelStructureFactory() override;


        /*!
         * Creates a ModelStructure attachment.
         * \param name  The name of the attachment.
         * \param localTransform    The transformation to apply to the attachment's pose after attaching to a ModelNode.
         * \return  A fully initialized attachment.
         */
        virtual ModelNodeAttachmentPtr createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform = Eigen::Matrix4f::Identity()) const override;

        // AbstractFactoryMethod
    public:
        /*!
         * \return "ModelStructure"
         */
        static std::string getName();
        static std::shared_ptr<ModelNodeAttachmentFactory> createInstance(void*);

    private:
        static SubClassRegistry registry;
    };
    typedef std::shared_ptr<ModelStructureFactory> ModelStructureFactoryPtr;
}
