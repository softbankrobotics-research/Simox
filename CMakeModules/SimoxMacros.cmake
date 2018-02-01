
# Build and helper macros

function(setupSimoxExternalLibraries)
  IF (Simox_VISUALIZATION)
    # we need to check for Qt
    IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
      #  if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
      #    MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
      #endif()
      file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
    ENDIF()
    
    if (Simox_USE_QT4)
        FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)
        else()
        FIND_PACKAGE(Qt5 5.5.0 COMPONENTS OpenGL Core Gui)
    endif()
  ENDIF()
  INCLUDE_DIRECTORIES(${Simox_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(SYSTEM ${Simox_EXTERNAL_INCLUDE_DIRS})
  ADD_DEFINITIONS( ${Simox_EXTERNAL_LIBRARY_FLAGS} )
  LINK_DIRECTORIES( ${Simox_LIBRARY_DIRS} )
endfunction()


function(VirtualRobotApplication name srcs incs)
    setupSimoxExternalLibraries()
    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
endfunction()


function(VirtualRobotQtApplication name srcs incs mocFiles uiFiles)
    setupSimoxExternalLibraries()

    if (Simox_USE_QT4)
        MESSAGE (STATUS "Qt4 Moc'ing: ${mocFiles}")
        MESSAGE (STATUS "Qt4 ui files: ${uiFiles}")
        # need this option to work around a qt/boost bug
        qt4_wrap_cpp(generatedMocFiles ${mocFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        qt4_wrap_ui(generatedUiFiles ${uiFiles})
    else()
        MESSAGE (STATUS "Qt5 Moc'ing: ${mocFiles}")
        MESSAGE (STATUS "Qt5 ui files: ${uiFiles}")
        # need this option to work around a qt/boost bug
        qt5_wrap_cpp(generatedMocFiles ${mocFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        qt5_wrap_ui(generatedUiFiles ${uiFiles})
    endif()

    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
endfunction()


function(VirtualRobotQtLibrary name srcs incs mocFiles uiFiles)
    setupSimoxExternalLibraries()

    if (Simox_USE_QT4)
        MESSAGE (STATUS "Qt4 Moc'ing: ${mocFiles}")
        MESSAGE (STATUS "Qt4 ui files: ${uiFiles}")
        # need this option to work around a qt/boost bug
        qt4_wrap_cpp(generatedMocFiles ${mocFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        qt4_wrap_ui(generatedUiFiles ${uiFiles})
    else()
        MESSAGE (STATUS "Qt5 Moc'ing: ${mocFiles}")
        MESSAGE (STATUS "Qt5 ui files: ${uiFiles}")
        # need this option to work around a qt/boost bug
        qt5_wrap_cpp(generatedMocFiles ${mocFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        qt5_wrap_ui(generatedUiFiles ${uiFiles})
    endif()

    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
    ################################## LIBRARY ##############################
    ADD_LIBRARY(${name} SHARED ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
endfunction()


function(SimoxApplication name srcs incs)
    VirtualRobotApplication("${name}" "${srcs}" "${incs}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtApplication name srcs incs mocFiles uiFiles)
    VirtualRobotQtApplication("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")  
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtLibrary name srcs incs mocFiles uiFiles)
    VirtualRobotQtLibrary("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()
