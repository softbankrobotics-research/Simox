/**
* @package    Gui
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#include "OgreViewerFactory.h"
#include "OgreViewer.h"

namespace SimoxGui
{

    OgreViewerFactory::OgreViewerFactory()
    {
    }


    OgreViewerFactory::~OgreViewerFactory()
    {
    }

    ViewerInterfacePtr OgreViewerFactory::createViewer(QWidget *parent)
    {
        OgreViewerPtr v(new OgreViewer(parent));
        return v;
    }


    /**
    * register this class in the super class factory
    */
    ViewerFactory::SubClassRegistry OgreViewerFactory::registry(OgreViewerFactory::getName(), &OgreViewerFactory::createInstance);


    /**
    * \return "ogre"
    */
    std::string OgreViewerFactory::getName()
    {
        return "ogre";
    }


    /**
    * \return new instance of OgreVisualizationFactory
    * if it has not already been called.
    */
    ViewerFactoryPtr OgreViewerFactory::createInstance(void*)
    {
        boost::shared_ptr<OgreViewerFactory> OgreFactory(new OgreViewerFactory());
        return OgreFactory;
    }

} // namespace SimoxGui
