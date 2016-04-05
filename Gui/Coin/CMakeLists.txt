set(INCLUDES
    CoinViewer.h
    ../ViewerInterface.h
    ../ViewerFactory.h
    CoinViewerFactory.h
)

set(SOURCES
    CoinViewer.cpp
    CoinViewerFactory.cpp
)

ADD_LIBRARY(SimoxCoinGui SHARED ${SOURCES} ${INCLUDES})
#TARGET_LINK_LIBRARIES()
