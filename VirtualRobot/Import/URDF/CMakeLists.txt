PROJECT (SIMOX_URDFREADER)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

include(SearchForTinyXML)
FIND_PACKAGE(urdfdom)

if (tinyxml_library AND URDF_FOUND)
    SET ( URDFIMPORT_SOURCES
            SimoxURDFFactory.cpp
            )
    SET ( URDFIMPORT_HEADERS
            SimoxURDFFactory.h
            )

    ADD_LIBRARY ( simox_urdfimport SHARED ${URDFIMPORT_SOURCES}  ${URDFIMPORT_HEADERS}  )

    target_include_directories(simox_urdfimport SYSTEM PUBLIC ${tinyxml_include_dirs})
    #arget_link_directories(${tinyxml_library_dirs})
    target_compile_definitions(simox_urdfimport PUBLIC ${tinyxml_cflags})
    target_compile_definitions(simox_urdfimport PUBLIC "-DTIXML_USE_STL")

    FIND_PACKAGE(simox REQUIRED)

    target_INCLUDE_DIRECTORIES (simox_urdfimport PUBLIC
	    ${URDFDOM_INCLUDE_DIRS}
	    ${URDFDOM_PARSER_INCLUDE_DIRS}
	    ${URDFDOM_HEADERS_INCLUDE_DIRS}
    )

    SET (CMAKE_SKIP_BUILD_RPATH FALSE)
    SET (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    SET (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


    ADD_EXECUTABLE (loadURDFobot loadURDFobot.cpp)
    target_compile_definitions(loadURDFobot PUBLIC "-DDEMO_BASE_PATH=\"${PROJECT_SOURCE_DIR}\"")
    
    TARGET_LINK_LIBRARIES (simox_urdfimport VirtualRobot ${URDFDOM_LIBRARIES})
    TARGET_LINK_LIBRARIES(loadURDFobot simox_urdfimport VirtualRobot ${URDFDOM_LIBRARIES})
endif()
