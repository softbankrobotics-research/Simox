/**
* @package    Gui
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#include "CoinViewerFactory.h"
#include "CoinViewer.h"

namespace SimoxGui
{
    AbstractViewerPtr CoinViewerFactory::createViewer(QWidget *parent) const
    {
        return AbstractViewerPtr(new CoinViewer(parent));
    }

} // namespace SimoxGui
