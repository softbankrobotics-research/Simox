set(INCLUDES
    CoinViewer.h
)

set(SOURCES
    CoinViewer.cpp
)

ADD_LIBRARY(SimoxCoinGui SHARED ${SOURCES} ${INCLUDES})
#TARGET_LINK_LIBRARIES()
