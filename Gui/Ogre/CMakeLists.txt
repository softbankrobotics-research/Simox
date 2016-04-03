set(INCLUDES
    OgreViewer.h
    OrbitCamera.h
    ../ViewerInterface.h
)

set(SOURCES
    OgreViewer.cpp
    OrbitCamera.cpp
)

ADD_LIBRARY(SimoxOgreGui SHARED ${SOURCES} ${INCLUDES})
#TARGET_LINK_LIBRARIES()
