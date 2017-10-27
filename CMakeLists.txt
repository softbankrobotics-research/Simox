PROJECT(Simox)

cmake_minimum_required(VERSION 2.8.3)
if(NOT ("${CMAKE_VERSION}" VERSION_LESS 2.8.12))
    cmake_policy(SET CMP0022 OLD) # avoid INTERFACE_LINK_LIBRARIES warnings
ENDIF()
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW) # qtmain add lib dependency (no effect in our case)
endif()

MESSAGE(STATUS "******************** Configuring Simox ************************")
set (Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR} CACHE STRING "Simox build directory")
#set (Simox_BUILD_VirtualRobot TRUE CACHE BOOL "Build Simox VirtualRobot")
set (Simox_BUILD_MotionPlanning TRUE CACHE BOOL "Build Simox Motion Planning library")
set (Simox_BUILD_GraspPlanning TRUE CACHE BOOL "Build Simox Grasp Planning library")
set (Simox_BUILD_Gui TRUE CACHE BOOL "Build Simox-Gui library")
set (Simox_BUILD_SimDynamics FALSE CACHE BOOL "Build Dynamic Simulation")

INCLUDE (config.cmake)


MESSAGE(STATUS "** Simox build directory: ${Simox_BUILD_DIRECTORY}")
add_subdirectory(VirtualRobot)
set (SIMOX_EXPORT_TARGET_LIST simox-virtualrobot)
list (APPEND Simox_LIBRARIES simox-virtualrobot)
list(APPEND SIMOX_EXPORT_TARGET_LIST ColCheckerPQP)
if (Simox_BUILD_MotionPlanning)
    add_subdirectory(MotionPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST simox-motionplanning)
list (APPEND Simox_LIBRARIES simox-motionplanning)
endif()

if (Simox_BUILD_GraspPlanning)
    add_subdirectory(GraspPlanning)
    list(APPEND SIMOX_EXPORT_TARGET_LIST simox-graspplanning)
    if (BUILD_qhull)
        list(APPEND SIMOX_EXPORT_TARGET_LIST simox-qhull)
    endif()
    list (APPEND Simox_LIBRARIES simox-graspplanning)
endif()

if (Simox_BUILD_SimDynamics)
    add_subdirectory(SimDynamics)
    list(APPEND SIMOX_EXPORT_TARGET_LIST simox-simdynamics)
    list (APPEND Simox_LIBRARIES simox-simdynamics)
    if (SimDynamics_BULLET_OpenGL)
        list(APPEND SIMOX_EXPORT_TARGET_LIST BulletOpenGLSupport)
    endif()

    list (APPEND Simox_EXTERNAL_LIBRARIES ${Simox_SimDynamics_EXTERNAL_LIBRARIES})
    list (APPEND Simox_EXTERNAL_INCLUDE_DIRS ${Simox_SimDynamics_INCLUDE_DIRS})
    set (Simox_EXTERNAL_LIBRARY_FLAGS "${Simox_EXTERNAL_LIBRARY_FLAGS} ${Simox_SimDynamics_COMPILE_FLAGS}")
endif()

  MESSAGE(STATUS "** Simox_EXTERNAL_INCLUDE_DIRS:${Simox_EXTERNAL_INCLUDE_DIRS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARY_DIRS:${Simox_EXTERNAL_LIBRARY_DIRS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARIES:${Simox_EXTERNAL_LIBRARIES}")
  MESSAGE(STATUS "** Simox_EXTERNAL_LIBRARY_FLAGS:${Simox_EXTERNAL_LIBRARY_FLAGS}")
  MESSAGE(STATUS "** Simox_EXTERNAL_CMAKE_INCLUDE:${Simox_EXTERNAL_CMAKE_INCLUDE}")

add_subdirectory(doc)

if (Simox_BUILD_Gui)

    add_subdirectory(Gui)
    
    if (Simox_USE_COIN_VISUALIZATION)
    	list(APPEND SIMOX_EXPORT_TARGET_LIST simox-gui-coin3d)
    	list (APPEND Simox_LIBRARIES simox-gui-coin3d)
    endif()

endif()


#######################################################################
# Setup for installation
#######################################################################

############################################
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

set(CONF_INCLUDE_DIRS "${INSTALL_INCLUDE_DIR}")
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


INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}
    COMPONENT ApplicationData
        FILES_MATCHING
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.stl"
        PATTERN "*.urdf"
        PATTERN "*.dae"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        )


# Install the export set for use with the install-tree
install(EXPORT SimoxTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)


#############################################################
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

#############################################################
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


