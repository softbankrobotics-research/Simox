
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

    ADD_LIBRARY(simox-gui-coin3d SHARED ${SOURCES} ${INCLUDES})
endif()
