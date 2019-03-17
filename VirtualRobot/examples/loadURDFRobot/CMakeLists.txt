PROJECT ( loadURDFRobot )

if (tinyxml_library AND URDF_FOUND)
  ADD_EXECUTABLE(${PROJECT_NAME} loadURDFobot.cpp)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

  TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE VirtualRobot ${URDFDOM_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PRIVATE "-DDEMO_BASE_PATH=\"${PROJECT_SOURCE_DIR}\"")
  
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

endif()


