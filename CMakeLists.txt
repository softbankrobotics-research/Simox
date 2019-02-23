cmake_minimum_required(VERSION 3.10)

#######################################################################################
PROJECT(Simox LANGUAGES CXX VERSION 2.3.73.0)
MESSAGE (STATUS "** Simox version: ${Simox_VERSION}")
#######################################################################################
#global config
# Set up build type

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
ENDIF()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${Simox_SOURCE_DIR}/CMakeModules)
MESSAGE (STATUS "** Module path: "  ${CMAKE_MODULE_PATH})

# we want a global automoc
FIND_PACKAGE(Qt5 5.5.0 COMPONENTS OpenGL Core Gui Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


FIND_PACKAGE(Boost COMPONENTS unit_test_framework REQUIRED)

#flags
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wpedantic)
#turn warnings about dangerous stuff into errors
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=maybe-uninitialized)
add_compile_options(-Werror=reorder)
add_compile_options(-Werror=parentheses)
#######################################################################################
#options
OPTION (Simox_USE_RBDL "Use RBDL" OFF)
OPTION (Simox_USE_URDF "Use URDF" OFF)
OPTION (Simox_USE_NLOPT "Use NLOPT" ON)
OPTION(BUILD_qhull "if on builds qhull external library" OFF)
OPTION(SimDynamics_USE_BULLET "Use Bullet Physics Engine" ON)
OPTION(Simox_USE_COIN_VISUALIZATION "Use Coin3D for visualization" ON)
OPTION(Simox_USE_OPENSCENEGRAPH_VISUALIZATION "Use OpenSceneGraph for visualization" OFF)
OPTION(Simox_COLLISION_CHECKER_PQP "Build and use the PQP Collision Detection library (located in ExternalDependencies)" ON)
#######################################################################################
MESSAGE(STATUS "******************** Configuring Simox ************************")
set (Simox_BUILD_Saba TRUE CACHE BOOL "Build Motion Planning library")
set (Simox_BUILD_GraspStudio TRUE CACHE BOOL "Build Grasp Planning library")
set (Simox_BUILD_SimDynamics TRUE CACHE BOOL "Build Dynamic Simulation")

MESSAGE (STATUS "** SIMOX BUILD VirtualRobot: TRUE (can't e turned off)")
MESSAGE (STATUS "** SIMOX BUILD Saba        : ${Simox_BUILD_Saba}")
MESSAGE (STATUS "** SIMOX BUILD GraspStudio : ${Simox_BUILD_GraspStudio}")
MESSAGE (STATUS "** SIMOX BUILD SimDynamics : ${Simox_BUILD_SimDynamics}")
############################# SETUP PATHS #############################
if(WIN32 AND NOT CYGWIN)
    set(INSTALL_CMAKE_DIR CMake             CACHE PATH "Installation directory for CMake files")
else()
    set(INSTALL_CMAKE_DIR share/Simox/cmake CACHE PATH "Installation directory for CMake files")
endif()

SET(Simox_LIB_DIR ${Simox_BINARY_DIR}/lib)
SET(Simox_BIN_DIR ${Simox_BINARY_DIR}/bin)
SET(Simox_DATA_DIR ${CurrentSimoxPath}/VirtualRobot/data)

MESSAGE (STATUS "** SIMOX LIB DIR: ${Simox_LIB_DIR}")
MESSAGE (STATUS "** SIMOX BIN DIR: ${Simox_BIN_DIR}")
MESSAGE (STATUS "** SIMOX DATA DIR: ${Simox_DATA_DIR}")

SET(Simox_INSTALL_LIB_DIR lib)
SET(Simox_INSTALL_BIN_DIR bin)
SET(Simox_INSTALL_DATA_DIR data)
SET(Simox_INSTALL_HEADER_DIR include)
MESSAGE (STATUS "** SIMOX INSTALL LIB DIR   : ${Simox_INSTALL_LIB_DIR}")
MESSAGE (STATUS "** SIMOX INSTALL BIN DIR   : ${Simox_INSTALL_BIN_DIR}")
MESSAGE (STATUS "** SIMOX INSTALL DATA DIR  : ${Simox_INSTALL_DATA_DIR}")
MESSAGE (STATUS "** SIMOX INSTALL HEADER DIR: ${Simox_INSTALL_HEADER_DIR}")

MESSAGE(STATUS "** Simox build directory: ${CMAKE_BINARY_DIR}")
########################### IDE settings ################################
# use virtual folders for grouping projects in IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

############################# Set OS specific options #############################
set(Simox_OS_compile_options)
IF(UNIX)
    # We are on Linux
    SET(Simox_TEST_DIR ${Simox_BIN_DIR}/tests)
    IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(Simox_OS_compile_options ${Simox_OS_compile_options} -fPIC)
    ENDIF()

    IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        MESSAGE(STATUS "Configuring Debug build")
        set(Simox_OS_compile_options ${Simox_OS_compile_options} -D_DEBUG)
    ELSE()
        MESSAGE(STATUS "Configuring Release build")
    ENDIF()

    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET(CMAKE_INSTALL_RPATH "${Simox_INSTALL_LIB_DIR}")
    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    # the RPATH to be used when installing, but only if it's not a system directory
    LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${Simox_INSTALL_LIB_DIR}" isSystemDir)
    IF("${isSystemDir}" STREQUAL "-1")
        SET(CMAKE_INSTALL_RPATH "${Simox_INSTALL_LIB_DIR}")
    ENDIF()

ELSE()
    # We are on Windows
    SET(Simox_TEST_DIR ${Simox_BIN_DIR})
    set(Simox_OS_compile_options
        -D_CRT_SECURE_NO_WARNINGS
        -DWIN32_LEAN_AND_MEAN # avoid errors with winsock / winsock2 includes
    )

    # On MSVC we compile with /MP flag (use multiple threads)
    IF(MSVC)
        set(Simox_OS_compile_options ${Simox_OS_compile_options} /MP)
    ENDIF()
ENDIF()
#######################################################################
# Setup for testing
ENABLE_TESTING()
INCLUDE(CTest)
MESSAGE(STATUS "** Test output directory: ${Simox_TEST_DIR}")
ADD_DEFINITIONS(-DSimox_TEST_DIR=\"${Simox_TEST_DIR}/\")

#######################################################################################
add_subdirectory(VirtualRobot)


#######################################################################################
set (SIMOX_EXPORT_TARGET_LIST VirtualRobot)
list (APPEND Simox_LIBRARIES VirtualRobot)
list(APPEND SIMOX_EXPORT_TARGET_LIST ColCheckerPQP)


#######################################################################################
if (Simox_BUILD_Saba)
    add_subdirectory(MotionPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST Saba)
list (APPEND Simox_LIBRARIES Saba)
endif()

#######################################################################################
if (Simox_BUILD_GraspStudio)
    add_subdirectory(GraspPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST GraspStudio)
    if (BUILD_qhull)
        list(APPEND SIMOX_EXPORT_TARGET_LIST simox-qhull)
    endif()
    list (APPEND Simox_LIBRARIES GraspStudio)
endif()

#######################################################################################
if (Simox_BUILD_SimDynamics)
    find_package(Bullet QUIET)
    if(BULLET_FOUND)
        add_subdirectory(SimDynamics)
        list(APPEND SIMOX_EXPORT_TARGET_LIST SimDynamics)
        list (APPEND Simox_LIBRARIES SimDynamics)
        if (SimDynamics_BULLET_OpenGL)
            list(APPEND SIMOX_EXPORT_TARGET_LIST BulletOpenGLSupport)
        endif()

    else()
        message("NOT building SimDynamics since bullet was not found")
    endif()
endif()

add_subdirectory(doc)


#######################################################################################
# Setup for installation
MESSAGE (STATUS "** Simox: Generating CMake files for build and install setup")

# Add all targets to the build-tree export set

export(TARGETS ${SIMOX_EXPORT_TARGET_LIST}
  FILE "${PROJECT_BINARY_DIR}/SimoxTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE Simox)

# Create the SimoxConfig.cmake and SimoxVersion files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
set(Simox_DATA_PATH ${Simox_DATA_DIR})
set(Simox_MACROS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/CMakeModules")

configure_file(CMakeModules/SimoxBuildTreeSettings.cmake.in
    "${PROJECT_BINARY_DIR}/SimoxBuildTreeSettings.cmake" @ONLY)

MESSAGE ("INSTALL_CMAKE_DIR:${INSTALL_CMAKE_DIR}")
# ... for the install tree
file(RELATIVE_PATH RELATIVE_INSTALL_DIR
     "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}"
     "${CMAKE_INSTALL_PREFIX}")

set(CONF_INCLUDE_DIRS "include")
set(Simox_DATA_PATH "${Simox_INSTALL_DATA_DIR}")
set(Simox_MACROS_INCLUDE_DIR "\${Simox_CMAKE_DIR}")

configure_file(CMakeModules/SimoxConfig.cmake.in
    "${PROJECT_BINARY_DIR}/SimoxConfig.cmake" @ONLY)

# ... for both
configure_file(CMakeModules/SimoxConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/SimoxConfigVersion.cmake" @ONLY)

# Install the SimoxBuildTreeSettings.cmake, SimoxConfig.cmake, and SimoxConfigVersion.cmake
install(FILES
    "${PROJECT_BINARY_DIR}/SimoxBuildTreeSettings.cmake"
    "${PROJECT_BINARY_DIR}/SimoxConfig.cmake"
    "${PROJECT_BINARY_DIR}/SimoxConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(FILES
    "${PROJECT_SOURCE_DIR}/CMakeModules/SimoxMacros.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}"
    COMPONENT ApplicationData
    )


# Install the export set for use with the install-tree
install(EXPORT SimoxTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)


#######################################################################################
# PACKAGING (DEB)

# git version not used any more!
find_package(Git QUIET)
if(GIT_FOUND)
    # output format <tag>-<commit-id>-dirty or <tag>-<commit-id> or <commit-id>
    execute_process(COMMAND ${GIT_EXECUTABLE} describe --long --tags --dirty --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE  GIT_EXIT_STATUS
        OUTPUT_VARIABLE SIMOX_PACKAGE_GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(NOT GIT_EXIT_STATUS EQUAL 0)
        message(WARNING "no git repository or unable to run git describe")
        set(Project_WC_REVISION "Unknown")
    else()
        # string(REGEX REPLACE "\n" "" SIMOX_PACKAGE_GIT_VERSION ${SIMOX_PACKAGE_GIT_VERSION})
        message(STATUS "simox git version: ${SIMOX_PACKAGE_GIT_VERSION}")
        set(Project_WC_REVISION ${SIMOX_PACKAGE_GIT_VERSION})
    endif()
endif()

IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    SET(CPACK_DEBIAN_ARCHITECTURE "amd64")
ELSE()
    SET(CPACK_DEBIAN_ARCHITECTURE "i386")
ENDIF()

INCLUDE(InstallRequiredSystemLibraries)
SET(CPACK_SET_DESTDIR "on")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
SET(CPACK_DEBIAN_PACKAGE_NAME "simox")
SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_DESCRIPTION "Simox - A Robot Simulation, Motion and Grasp Planning Toolbox")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Simox - A Robot Simulation, Motion and Grasp Planning Toolbox")
SET(CPACK_PACKAGE_VENDOR "KIT")
SET(CPACK_PACKAGE_CONTACT "Nikolaus Vahrenkamp <vahrenkamp@kit.edu>")
SET(CPACK_PACKAGE_VERSION_MAJOR "${Simox_MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${Simox_MINOR_VERSION}")
#SET(CPACK_PACKAGE_VERSION_PATCH "${Simox_PATCH_VERSION}-r${Project_WC_REVISION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${Simox_PATCH_VERSION}")
SET(CPACK_PACKAGE_FILE_NAME "simox_${Project_WC_REVISION}_${CPACK_DEBIAN_ARCHITECTURE}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "simox_${Project_WC_REVISION}")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#GET_PROPERTY(result GLOBAL ENABLED_FEATURES)
#message(" enabled features: ${result}")
set(CPACK_COMPONENT_APPLICATIONS_DISPLAY_NAME "Simox Examples")
set(CPACK_COMPONENT_APPLICATIONDATA_DISPLAY_NAME "Simox Data")
set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Simox Libraries")
set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "Simox C++ Headers")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "libbullet-dev, libbulletmultithreaded2.82, libqhull-dev, libnlopt-dev")

SET(CPACK_COMPONENTS_ALL Applications Headers Libraries ApplicationData)
INCLUDE(CPack)

#######################################################################################
# STYLE

FIND_PACKAGE(AStyle QUIET)

if(AStyle_FOUND)
    add_custom_target(astyle
                      COMMAND ${AStyle_EXECUTABLE} --options="${PROJECT_SOURCE_DIR}/armarx.astylerc" --recursive --preserve-date --formatted "*.cpp" "*.h" "*.hpp"
                      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
                      COMMENT "Formatting source files in: ${PROJECT_SOURCE_DIR}")
else()
    add_custom_target(astyle
                      COMMENT "Unable to format source files: AStyle was not found!")
endif()


