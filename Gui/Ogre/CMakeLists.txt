PROJECT ( SimoxOgreGui )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW) # qtmain add lib dependency (no effect in our case)
endif()

MESSAGE (STATUS "***** CONFIGURING Simox project SimoxOgreGui *****")

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
        ../ViewerFactory.h
        OgreViewerFactory.h
    )

    set(SOURCES
        OgreViewer.cpp
        OrbitCamera.cpp
        OgreViewerFactory.cpp
        ${generatedMocFiles}
    )
	MESSAGE("OGRE_RenderSystem_GL_LIBRARY_REL: ${OGRE_RenderSystem_GL_LIBRARY_REL}")
	MESSAGE("OGRE_RenderSystem_GL_BINARY_REL: ${OGRE_RenderSystem_GL_REL}")
	MESSAGE("OGRE_RenderSystem_GL_BINARY_DBG: ${OGRE_RenderSystem_GL_DBG}")

	#set (OGRE_RENDER_SYSTEM ${OGRE_RenderSystem_GL_LIBRARY_REL})
	if (OGRE_RenderSystem_GL_REL)
		MESSAGE("using OGRE_RenderSystem_GL_BINARY_REL: ${OGRE_RenderSystem_GL_REL}")
		set (OGRE_RENDER_SYSTEM ${OGRE_RenderSystem_GL_REL})
	elseif (OGRE_RenderSystem_GL_DBG)
		MESSAGE("using OGRE_RenderSystem_GL_BINARY_DBG: ${OGRE_RenderSystem_GL_DBG}")
		set (OGRE_RENDER_SYSTEM ${OGRE_RenderSystem_GL_DBG})
	elseif (OGRE_RenderSystem_GL_LIBRARY_REL)
		MESSAGE("using OGRE_RenderSystem_GL_LIBRARY_REL: ${OGRE_RenderSystem_GL_LIBRARY_REL}")
		set (OGRE_RENDER_SYSTEM ${OGRE_RenderSystem_GL_LIBRARY_REL})
	elseif (OGRE_RenderSystem_GL_LIBRARY_DBG)
		MESSAGE("using OGRE_RenderSystem_GL_LIBRARY_DBG: ${OGRE_RenderSystem_GL_LIBRARY_DBG}")
		set (OGRE_RENDER_SYSTEM ${OGRE_RenderSystem_GL_LIBRARY_DBG})
	else()
		MESSAGE("OGRE Rendersystem not defined...")
	endif()
	MESSAGE("OGRE_RENDER_SYSTEM: ${OGRE_RENDER_SYSTEM}")
	ADD_DEFINITIONS(-DOGRE_RENDERING_PLUGIN=\"${OGRE_RENDER_SYSTEM}\")
    ADD_LIBRARY(SimoxOgreGui SHARED ${SOURCES} ${INCLUDES})
    TARGET_LINK_LIBRARIES(SimoxOgreGui ${OGRE_LIBRARIES} VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
    #SET_TARGET_PROPERTIES(SimoxOgreGui PROPERTIES COMPILE_FLAGS -DOGRE_RENDERING_PLUGIN=\"${OGRE_RENDER_SYSTEM}\")

	# .DLL path
	SET_TARGET_PROPERTIES(SimoxOgreGui PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
	# .so path
	SET_TARGET_PROPERTIES(SimoxOgreGui PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
	# .lib path (this is needed for setting the DLL-import library path on windows)
	SET_TARGET_PROPERTIES(SimoxOgreGui PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})

	#######################################################################################
	############################ Setup for installation ###################################
	#######################################################################################

	install(TARGETS ${PROJECT_NAME}
	  # IMPORTANT: Add the library to the "export-set"
	  EXPORT SimoxTargets
	  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
	  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
	  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
	  COMPONENT dev)

	INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
		COMPONENT Headers
			FILES_MATCHING PATTERN "*.h"
			PATTERN ".svn" EXCLUDE
			PATTERN ".git" EXCLUDE
			PATTERN "CMakeModules" EXCLUDE
			PATTERN "tests" EXCLUDE
			PATTERN "build*" EXCLUDE
			PATTERN "data" EXCLUDE
			PATTERN "examples" EXCLUDE
		)
	MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimoxOgreGui *****\n")

endif()
