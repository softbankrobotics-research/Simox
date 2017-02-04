PROJECT ( ReachabilityInversionDemo )

INCLUDE(${Simox_DIR}/CMakeModules/SimoxMacros.cmake)

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)

	SET(DEMO_BASE_DIR ${PROJECT_SOURCE_DIR})
	ADD_DEFINITIONS(-DDEMO_BASE_DIR="${DEMO_BASE_DIR}")

	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/InvReachDemo.cpp 
                            ${PROJECT_SOURCE_DIR}/InvReachWindow.cpp
                            ${PROJECT_SOURCE_DIR}/setupdialog.cpp)

        FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/InvReachWindow.h
                            ${PROJECT_SOURCE_DIR}/setupdialog.h)
  
  ################################## moc'ing ##############################
  set(GUI_MOC_HDRS
      ${PROJECT_SOURCE_DIR}/InvReachWindow.h
      ${PROJECT_SOURCE_DIR}/setupdialog.h
  )
  
  set(GUI_UIS
      ${PROJECT_SOURCE_DIR}/InvReachDemo.ui
      ${PROJECT_SOURCE_DIR}/setupdialog.ui
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
