PROJECT (VirtualRobotImportUrdf)

include(SearchForTinyXML)
FIND_PACKAGE(urdfdom)
if (tinyxml_library AND URDF_FOUND)
    message(STATUS "Building VirtualRobotImportUrdf")

    include_directories(SYSTEM ${tinyxml_include_dirs})
    link_directories(${tinyxml_library_dirs})
    add_definitions(${tinyxml_cflags})
    add_definitions("-DTIXML_USE_STL")

    INCLUDE_DIRECTORIES (
	    ${URDFDOM_INCLUDE_DIRS}
	    ${URDFDOM_PARSER_INCLUDE_DIRS}
	    ${URDFDOM_HEADERS_INCLUDE_DIRS}
    )

    SET ( SOURCES SimoxURDFFactory.cpp)
    SET ( HEADERS SimoxURDFFactory.h  )

    ADD_LIBRARY ( VirtualRobotImportUrdf SHARED ${SOURCES}  ${HEADERS}  )
    
    TARGET_LINK_LIBRARIES (VirtualRobotImportUrdf VirtualRobot ${URDFDOM_LIBRARIES})
else()
    if (NOT tinyxml_library)
    message(STATUS "Not building VirtualRobotImportUrdf: tinyxml not found")
    endif()
    if (NOT URDF_FOUND)
    message(STATUS "Not building VirtualRobotImportUrdf: urdfdom not found")
    endif()
endif()
