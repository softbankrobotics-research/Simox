#include "ViewerFactory.h"

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "Coin/CoinViewerFactory.h"

    using GlobalFactory = SimoxGui::CoinViewerFactory;
#endif

SimoxGui::ViewerFactoryPtr SimoxGui::ViewerFactory::getInstance()
{
    static ViewerFactoryPtr instance(new GlobalFactory);
    return instance;
}
