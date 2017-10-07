#ifndef MODELSTRUCTUREFACTORY_H
#define MODELSTRUCTUREFACTORY_H

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
        virtual ~ModelStructureFactory();


        /*!
         * Create a new attachment.
         *
         * @return The new attachment.
         */
        virtual ModelNodeAttachmentPtr createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform = Eigen::Matrix4f::Identity(), std::string visualizationType = "");

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



#endif // MODELSTRUCTUREFACTORY_H
