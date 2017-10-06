/**
* @package    Gui
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#include "CoinViewerFactory.h"
#include "CoinViewer.h"

namespace SimoxGui
{

    CoinViewerFactory::CoinViewerFactory()
    {
    }


    CoinViewerFactory::~CoinViewerFactory()
    {
    }

    ViewerInterfacePtr CoinViewerFactory::createViewer(QWidget *parent)
    {
        CoinViewerPtr v(new CoinViewer(parent));
        return v;
    }


    /**
    * register this class in the super class factory
    */
    ViewerFactory::SubClassRegistry CoinViewerFactory::registry(CoinViewerFactory::getName(), &CoinViewerFactory::createInstance);


    /**
    * \return "inventor"
    */
    std::string CoinViewerFactory::getName()
    {
        return "inventor";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    ViewerFactoryPtr CoinViewerFactory::createInstance(void*)
    {
        std::shared_ptr<CoinViewerFactory> CoinFactory(new CoinViewerFactory());
        return CoinFactory;
    }

} // namespace SimoxGui
