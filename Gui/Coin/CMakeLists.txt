set(INCLUDES
    CoinViewer.h
    ../ViewerInterface.h
    ../ViewerFactory.h
    ../ViewerSelection.h
    CoinViewerFactory.h
)

set(SOURCES
    CoinViewer.cpp
    CoinViewerFactory.cpp
)


INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

ADD_LIBRARY(simox-gui-coin3d SHARED ${SOURCES} ${INCLUDES})
#TARGET_LINK_LIBRARIES()
