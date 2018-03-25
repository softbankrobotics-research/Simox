#include "ViewerFactory.h"

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "Coin/CoinViewerFactory.h"

    using GlobalFactory = SimoxGui::CoinViewerFactory;
#elif Simox_USE_QT3D_VISUALIZATION
    #include "Qt3D/Qt3DViewerFactory.h"

    using GlobalFactory = SimoxGui::Qt3DViewerFactory;
#endif

SimoxGui::ViewerFactoryPtr SimoxGui::ViewerFactory::getInstance()
{
    static ViewerFactoryPtr instance(new GlobalFactory);
    return instance;
}
