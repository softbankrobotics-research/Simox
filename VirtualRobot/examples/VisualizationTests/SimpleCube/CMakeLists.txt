PROJECT ( SimpleCube )

INCLUDE(${Simox_DIR}/CMakeModules/SimoxMacros.cmake)

IF(Simox_VISUALIZATION AND Simox_GUI)

        # the variable "demo_SRCS" contains all .cpp files of this project
        FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/SimpleCube.cpp ${PROJECT_SOURCE_DIR}/SimpleCubeWindow.cpp)
        FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/SimpleCubeWindow.h)

  ################################## moc'ing ##############################
  set(GUI_MOC_HDRS
      ${PROJECT_SOURCE_DIR}/SimpleCubeWindow.h
  )

  set(GUI_UIS
      ${PROJECT_SOURCE_DIR}/SimpleCube.ui
  )

        # create the executable
  VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

  #######################################################################################
  ############################ Setup for installation ###################################
  #######################################################################################

  install(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    COMPONENT dev)

  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " ${INSTALL_BIN_DIR})

ENDIF()
