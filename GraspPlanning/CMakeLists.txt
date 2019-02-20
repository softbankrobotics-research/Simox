PROJECT ( GraspStudio )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW) # qtmain add lib dependency (no effect in our case)
endif()

MESSAGE (STATUS "***** CONFIGURING Simox project GraspStudio *****")

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

MACRO(ADD_GRASPSTUDIO_TEST TEST_NAME)
        if (NOT Boost_USE_STATIC_LIBS)
            ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
        endif (NOT Boost_USE_STATIC_LIBS)
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
        TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot Saba GraspStudio ${Simox_EXTERNAL_LIBRARIES})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "GraspStudio Tests")
        ADD_TEST(NAME Saba_${TEST_NAME}
                 COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_GRASPSTUDIO_TEST)


########### QHULL #################
OPTION(BUILD_qhull "if on builds qhull external library" OFF)
IF(NOT BUILD_qhull)
    find_package(QHULL QUIET)
    IF (NOT QHULL_FOUND)
		MESSAGE (STATUS "Could not find QHULL, building internal version simox-qhull")
		SET (BUILD_qhull ON)
		SET (BUILD_qhull ON PARENT_SCOPE)
    ENDIF()
ENDIF()

IF(BUILD_qhull)
    ADD_SUBDIRECTORY(ExternalDependencies/qhull-2003.1)
    set(QHULL_LIBRARIES simox-qhull)
    include_directories("ExternalDependencies/qhull-2003.1/include/")
else()
    find_package(QHULL REQUIRED)
    include_directories(${QHULL_INCLUDE_DIRS})
endif()
########### END QHULL #################


SET(SOURCES
ConvexHullGenerator.cpp
ContactConeGenerator.cpp
ApproachMovementGenerator.cpp
ApproachMovementSurfaceNormal.cpp
MeshConverter.cpp
GraspPlanner/GraspPlanner.cpp
GraspPlanner/GenericGraspPlanner.cpp
GraspQuality/GraspEvaluationPoseUncertainty.cpp
GraspQuality/GraspQualityMeasure.cpp
GraspQuality/GraspQualityMeasureWrenchSpace.cpp
GraspQuality/GraspQualityMeasureWrenchSpaceNotNormalized.cpp
Visualization/ConvexHullVisualization.cpp
)

SET(INCLUDES
GraspStudio.h
ConvexHullGenerator.h
ContactConeGenerator.h
ApproachMovementGenerator.h
ApproachMovementSurfaceNormal.h
MeshConverter.h
GraspPlanner/GraspPlanner.h
GraspPlanner/GenericGraspPlanner.h
GraspPlanner/GraspPlannerEvaluation.h
GraspQuality/GraspEvaluationPoseUncertainty.h
GraspQuality/GraspQualityMeasure.h
GraspQuality/GraspQualityMeasureWrenchSpace.h
GraspQuality/GraspQualityMeasureWrenchSpaceNotNormalized.h
Visualization/ConvexHullVisualization.h
)


if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.h
	)
endif ()


include_directories(SYSTEM ${Simox_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})


ADD_LIBRARY (GraspStudio SHARED ${SOURCES} ${INCLUDES})

TARGET_LINK_LIBRARIES (GraspStudio PUBLIC Saba VirtualRobot ${Simox_EXTERNAL_LIBRARIES} ${QHULL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})

if(Simox_BUILD_EXAMPLES)
    # include examples
    ADD_SUBDIRECTORY(examples/)
endif()

# include unit tests
#ADD_SUBDIRECTORY(tests/)


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################


install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
  COMPONENT dev)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)
	

MESSAGE (STATUS "***** Finished CONFIGURING Simox project GraspStudio *****\n")

