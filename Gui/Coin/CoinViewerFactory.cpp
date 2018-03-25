/**
* @package    Gui
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#include "CoinViewerFactory.h"
#include "CoinViewer.h"

namespace SimoxGui
{
    ViewerInterfacePtr CoinViewerFactory::createViewer(QWidget *parent) const
    {
        return ViewerInterfacePtr(new CoinViewer(parent));
    }

} // namespace SimoxGui
