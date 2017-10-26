PROJECT ( simox-gui-coin3d )

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

MESSAGE (STATUS "\n***** CONFIGURING project simox-gui *****")


# -----------------------------------------------------------------------------------
# Adding the UI files to the library
if (Simox_USE_COIN_VISUALIZATION)
    MESSAGE (STATUS " ** Adding GraspEditor gui")
    set(UI
        Grasping/GraspEditor/GraspEditor.ui
    )

  if (Simox_USE_QT4)
        QT4_WRAP_UI(UI_HEADERS ${UI})
        QT4_WRAP_CPP(MOC_SRC
            Grasping/GraspEditor/GraspEditorWindow.h
        )
    #QT4_AUTOMOC ( "Grasping/GraspEditor/GraspEditorWindow.cpp" )
    else()
        QT5_WRAP_UI(UI_HEADERS ${UI})
        QT5_WRAP_CPP(MOC_SRC
            Grasping/GraspEditor/GraspEditorWindow.h
        )
  endif()
  MESSAGE (STATUS " ** UI_HEADERS:${UI_HEADERS}")
  MESSAGE (STATUS " ** MOC_SRC:${MOC_SRC}")


    set(INCLUDES ${INCLUDES} ${UI_HEADERS})

    SET(SOURCES ${SOURCES} ${MOC_SRC} "Grasping/GraspEditor/GraspEditorWindow.cpp")
    SET(INCLUDES ${INCLUDES} "Grasping/GraspEditor/GraspEditorWindow.h")
endif()


if (Simox_USE_COIN_VISUALIZATION)
    list(APPEND INCLUDES
        Coin/CoinViewer.h
        SimoxGuiImportExport.h
        ViewerInterface.h
        ViewerFactory.h
        ViewerSelection.h
        Coin/CoinViewerFactory.h
    )

    list(APPEND SOURCES
        Coin/CoinViewer.cpp
        Coin/CoinViewerFactory.cpp
    )

    MESSAGE (STATUS " ** INCLUDES:${INCLUDES}")
    MESSAGE (STATUS " ** SOURCES:${SOURCES}")

    INCLUDE (Cxx11Test)
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

    ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${INCLUDES})
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} simox-virtualrobot ${Simox_EXTERNAL_LIBRARIES})

    MESSAGE (STATUS "simox-gui bin dir: ${Simox_BIN_DIR}")
    MESSAGE (STATUS "simox-gui lib dir: ${Simox_LIB_DIR}")
    # .DLL path
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
    # .so path
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
    # .lib path (this is needed for setting the DLL-import library path on windows)
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


endif()


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


MESSAGE (STATUS "***** Finished CONFIGURING project simox-gui *****\n")

