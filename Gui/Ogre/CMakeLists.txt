find_package(OGRE QUIET)

if(OGRE_FOUND)

    set(GUI_MOC_HDRS
        OgreViewer.h
    )

    if(Simox_USE_QT4)
        MESSAGE (STATUS "Qt4 Moc'ing: ${GUI_MOC_HDRS}")
        # need this option to work around a qt/boost bug
        qt4_wrap_cpp(generatedMocFiles ${GUI_MOC_HDRS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    else()
        MESSAGE (STATUS "Qt5 Moc'ing: ${GUI_MOC_HDRS}")
        # need this option to work around a qt/boost bug
        qt5_wrap_cpp(generatedMocFiles ${GUI_MOC_HDRS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    endif()

    set(INCLUDES
        OgreViewer.h
        OrbitCamera.h
        ../ViewerInterface.h
    )

    set(SOURCES
        OgreViewer.cpp
        OrbitCamera.cpp
        ${generatedMocFiles}
    )

    ADD_LIBRARY(SimoxOgreGui SHARED ${SOURCES} ${INCLUDES})
    TARGET_LINK_LIBRARIES(SimoxOgreGui ${OGRE_LIBRARIES})
    SET_TARGET_PROPERTIES(SimoxOgreGui PROPERTIES COMPILE_FLAGS -DOGRE_RENDERING_PLUGIN='"${OGRE_RenderSystem_GL_LIBRARY_REL}"')

endif()
