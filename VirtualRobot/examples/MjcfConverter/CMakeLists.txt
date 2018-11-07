project(MjcfConverter)

find_package(TinyXML2)
if (${TINYXML2_FOUND})
    include_directories(${TINYXML2_INCLUDE_DIR})
endif()
    

set(SOURCES
    main.cpp
    MjcfConverter.cpp
    MjcfDocument.cpp
)

set(HEADERS
    MjcfConverter.h
    MjcfDocument.h
)


add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

target_link_libraries(${PROJECT_NAME} VirtualRobot ${TINYXML2_LIBRARIES})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

install(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    COMPONENT dev)

message( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
message( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " ${INSTALL_BIN_DIR})

