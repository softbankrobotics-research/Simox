PROJECT ( RobotViewerOSG )

INCLUDE(${Simox_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake)

IF(Simox_VISUALIZATION AND Simox_USE_OPENSCENEGRAPH_VISUALIZATION)


	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RobotViewer.cpp ${PROJECT_SOURCE_DIR}/showRobotWindow.cpp)# osgViewerWidget.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/showRobotWindow.h)# osgViewerWidget.h)


  ################################## moc'ing ##############################
  set(GUI_MOC_HDRS
      ${PROJECT_SOURCE_DIR}/showRobotWindow.h

  )
  #${PROJECT_SOURCE_DIR}/osgViewerWidget.h

  set(GUI_UIS
      ${PROJECT_SOURCE_DIR}/RobotViewer.ui
  )
  
  VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
   
  #######################################################################################
  ############################ Setup for installation ###################################
  #######################################################################################

  install(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION bin COMPONENT bin
    COMPONENT dev)

  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " bin)
ENDIF()
