/**
* @package    Gui
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/

#include "CoinViewerFactory.h"
#include "CoinViewer.h"
#include "../CameraConfiguration.h"

namespace SimoxGui
{
    AbstractViewerPtr CoinViewerFactory::createViewer(QWidget *parent, const std::shared_ptr<std::recursive_mutex> &m) const
    {
        return AbstractViewerPtr(new CoinViewer(parent, m));
    }

    CameraConfigurationPtr CoinViewerFactory::createCameraConfiguration() const
    {
        return CameraConfigurationPtr(new CameraConfiguration);
    }

} // namespace SimoxGui
