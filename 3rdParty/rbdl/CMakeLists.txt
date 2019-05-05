#CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

SET ( RBDL_VERSION_MAJOR 2 )
SET ( RBDL_VERSION_MINOR 6 )
SET ( RBDL_VERSION_PATCH 0 )
SET ( RBDL_VERSION 
	${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET ( RBDL_SO_VERSION 
	${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET (PROJECT_VERSION ${RBDL_VERSION})
PROJECT (rbdl VERSION ${RBDL_VERSION})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )



INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}/include
)

# SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

#INCLUDE(GNUInstallDirs)

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
		LINKER_LANGUAGE CXX
	)

add_compile_options(-Wno-error=sign-compare)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-variable)






# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()





# Options
SET (RBDL_BUILD_STATIC_DEFAULT OFF)
IF (MSVC)
  SET (RBDL_BUILD_STATIC_DEFAULT ON)
ENDIF (MSVC)

OPTION (RBDL_BUILD_STATIC "Build statically linked library (otherwise dynamiclly linked)" ${RBDL_BUILD_STATIC_DEFAULT})
OPTION (RBDL_BUILD_TESTS "Build the test executables" OFF)
OPTION (RBDL_ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" OFF)
OPTION (RBDL_USE_SIMPLE_MATH "Use slow math instead of the fast Eigen3 library (faster compilation)" OFF)
OPTION (RBDL_STORE_VERSION "Enable storing of version information in the library (requires build from valid repository)" OFF)
OPTION (RBDL_BUILD_ADDON_URDFREADER "Build the (experimental) urdf reader" OFF)
OPTION (RBDL_BUILD_ADDON_BENCHMARK "Build the benchmarking tool" OFF)
OPTION (RBDL_BUILD_ADDON_LUAMODEL "Build the lua model reader" OFF)
OPTION (RBDL_BUILD_PYTHON_WRAPPER "Build experimental python wrapper" OFF)
OPTION (RBDL_BUILD_ADDON_GEOMETRY "Build the geometry library" OFF)
OPTION (RBDL_BUILD_ADDON_MUSCLE "Build the muscle library" OFF)

# Addons
IF (RBDL_BUILD_ADDON_URDFREADER)
  ADD_SUBDIRECTORY ( addons/urdfreader )
ENDIF (RBDL_BUILD_ADDON_URDFREADER)

IF (RBDL_BUILD_ADDON_BENCHMARK)
  ADD_SUBDIRECTORY ( addons/benchmark )
ENDIF (RBDL_BUILD_ADDON_BENCHMARK)

IF (RBDL_BUILD_ADDON_LUAMODEL)
  ADD_SUBDIRECTORY ( addons/luamodel )
ENDIF (RBDL_BUILD_ADDON_LUAMODEL)

IF(RBDL_BUILD_ADDON_MUSCLE)
  SET(RBDL_BUILD_ADDON_GEOMETRY ON CACHE BOOL "Build the geometry library" FORCE)
  ADD_SUBDIRECTORY ( addons/muscle )
  IF(RBDL_BUILD_TESTS)
  	ADD_SUBDIRECTORY ( addons/muscle/tests ) 
  ENDIF(RBDL_BUILD_TESTS)
ENDIF(RBDL_BUILD_ADDON_MUSCLE)


IF(RBDL_BUILD_ADDON_GEOMETRY)
  ADD_SUBDIRECTORY ( addons/geometry )
  IF(RBDL_BUILD_TESTS)
  	ADD_SUBDIRECTORY ( addons/geometry/tests ) 
  ENDIF(RBDL_BUILD_TESTS)
ENDIF(RBDL_BUILD_ADDON_GEOMETRY)



IF (RBDL_BUILD_TESTS)
 ADD_SUBDIRECTORY ( tests )
ENDIF (RBDL_BUILD_TESTS)

# Source files for RBDL
SET ( RBDL_SOURCES 
        src/rbdl/rbdl_version.cc
        src/rbdl/rbdl_mathutils.cc
        src/rbdl/rbdl_utils.cc
        src/rbdl/Constraints.cc
        src/rbdl/Dynamics.cc
        src/rbdl/Logging.cc
        src/rbdl/Joint.cc
        src/rbdl/Model.cc
        src/rbdl/Kinematics.cc
	)

IF (MSVC AND NOT RBDL_BUILD_STATIC)	
  MESSAGE (FATAL_ERROR, "Compiling RBDL as a DLL currently not supported. Please enable RBDL_BUILD_STATIC.")
ENDIF (MSVC AND NOT RBDL_BUILD_STATIC)	
	
# Static / dynamic builds
IF (RBDL_BUILD_STATIC)
  ADD_LIBRARY ( rbdl-static STATIC ${RBDL_SOURCES} )
  IF (NOT WIN32)
    SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES PREFIX "lib")
  ENDIF (NOT WIN32)
  SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES OUTPUT_NAME "rbdl")

	IF (RBDL_BUILD_ADDON_LUAMODEL)
		TARGET_LINK_LIBRARIES ( rbdl-static
			rbdl_luamodel-static
			)
	ENDIF (RBDL_BUILD_ADDON_LUAMODEL)

	INSTALL (TARGETS rbdl-static
	  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	)
ELSE (RBDL_BUILD_STATIC)
        ADD_LIBRARY ( rbdl SHARED ${RBDL_SOURCES} )
        SET_TARGET_PROPERTIES ( rbdl PROPERTIES
		VERSION ${RBDL_VERSION}
		SOVERSION ${RBDL_SO_VERSION}
		)

        INSTALL (TARGETS rbdl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		)
ENDIF (RBDL_BUILD_STATIC)


# Find and use the system's Eigen3 library

FIND_PACKAGE (Eigen3 REQUIRED)
target_include_directories(rbdl SYSTEM PUBLIC ${Eigen3_INCLUDE_DIR})

IF (RBDL_STORE_VERSION)
	# Set versioning information that can be queried during runtime
	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "id -i"
		OUTPUT_VARIABLE RBDL_BUILD_REVISION)
	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "branch"
		OUTPUT_VARIABLE RBDL_BUILD_BRANCH)
	SET (RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE (RBDL_STORE_VERSION)
	SET (RBDL_BUILD_REVISION "unknown")
	SET (RBDL_BUILD_BRANCH "unknown")
	SET (RBDL_BUILD_TYPE "unknown")
ENDIF (RBDL_STORE_VERSION)

CONFIGURE_FILE (
	"${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/rbdl_config.h.cmake" 
	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h"
	)
    target_include_directories(rbdl PUBLIC
        $<BUILD_INTERFACE:${Simox_SOURCE_DIR}/3rdParty/rbdl/include>
        $<BUILD_INTERFACE:${Simox_BINARY_DIR}/3rdParty/rbdl/include>
        $<INSTALL_INTERFACE:include>
    )

# Python wrapper
IF (RBDL_BUILD_PYTHON_WRAPPER)
	add_subdirectory ( python )

ENDIF (RBDL_BUILD_PYTHON_WRAPPER)


set_target_properties(rbdl PROPERTIES SOURCES "${RBDL_SOURCES}")


# Installation
FILE ( GLOB headers 
	${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/*.h
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h
	)

    INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
          COMPONENT Headers
            FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
            PATTERN ".svn" EXCLUDE
            PATTERN ".git" EXCLUDE
            PATTERN "CMakeModules" EXCLUDE
            PATTERN "tests" EXCLUDE
            PATTERN "build*" EXCLUDE
            PATTERN "data" EXCLUDE
            PATTERN "examples" EXCLUDE
          )
#INSTALL ( FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl )

# Setup of SimpleMath install settings
IF (RBDL_USE_SIMPLE_MATH)
	INSTALL ( DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/SimpleMath"
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl
	)
ENDIF (RBDL_USE_SIMPLE_MATH)

message("Exporting ${PROJECT_NAME}")
INSTALL(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  ARCHIVE DESTINATION lib COMPONENT library
  COMPONENT dev)


# pkg-config
CONFIGURE_FILE (
  ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.pc.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pc @ONLY
  )
INSTALL (
  FILES  ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
  )


#export(PACKAGE rbdl)

## Packaging
#SET(CPACK_GENERATOR "DEB")
#SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Martin Felis <martin@fysx.org>")
#SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "the Rigid Body Dynamics Library (RBDL)")
#SET(CPACK_PACKAGE_VENDOR "Martin Felis")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
#SET(CPACK_PACKAGE_VERSION_MAJOR ${RBDL_VERSION_MAJOR})
#SET(CPACK_PACKAGE_VERSION_MINOR ${RBDL_VERSION_MINOR})
#SET(CPACK_PACKAGE_VERSION_PATCH ${RBDL_VERSION_PATCH})
#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CPACK_PACKAGE ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
#SET(CPACK_PACKAGE_FILE_NAME "rbdl-${CMAKE_LIBRARY_ARCHITECTURE}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
#SET(CPACK_PACKAGE_EXECUTABLES "rbdl_luamodel_util;RBDL Lua Model Utility")

#INCLUDE(CPack)
