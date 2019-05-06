PROJECT ( GraspStudio )

MESSAGE (STATUS "***** CONFIGURING Simox project GraspStudio *****")


MACRO(ADD_GRASPSTUDIO_TEST TEST_NAME)
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
        if (NOT Boost_USE_STATIC_LIBS)
            target_compile_definitions(${TEST_NAME} PRIVATE -DBOOST_TEST_DYN_LINK)
        endif ()
        TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot Saba GraspStudio boost_unit_test_framework)
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "GraspStudio Tests")
        ADD_TEST(NAME Saba_${TEST_NAME}
                 COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_GRASPSTUDIO_TEST)


########### QHULL #################
find_package(QHULL REQUIRED)
########### END QHULL #################


SET(SOURCES
    ApproachMovementGenerator.cpp
    ApproachMovementSurfaceNormal.cpp
    ContactConeGenerator.cpp
    ConvexHullGenerator.cpp
    MeshConverter.cpp
    
    GraspPlanner/GenericGraspPlanner.cpp
    GraspPlanner/GraspPlanner.cpp
    GraspPlanner/GraspPlannerEvaluation.cpp
    
    GraspQuality/GraspEvaluationPoseUncertainty.cpp
    GraspQuality/GraspQualityMeasure.cpp
    GraspQuality/GraspQualityMeasureWrenchSpace.cpp
    GraspQuality/GraspQualityMeasureWrenchSpaceNotNormalized.cpp
    
    Visualization/ConvexHullVisualization.cpp
)

SET(INCLUDES
    GraspStudio.h
    
    ApproachMovementGenerator.h
    ApproachMovementSurfaceNormal.h
    ContactConeGenerator.h
    ConvexHullGenerator.h
    MeshConverter.h
    
    GraspPlanner/GenericGraspPlanner.h
    GraspPlanner/GraspPlanner.h
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

ADD_LIBRARY (GraspStudio SHARED ${SOURCES} ${INCLUDES})

IF(BUILD_qhull)
    target_include_directories(GraspStudio PUBLIC "ExternalDependencies/qhull-2003.1/include/")
else()
    target_include_directories(GraspStudio PUBLIC ${QHULL_INCLUDE_DIRS})
endif()

TARGET_LINK_LIBRARIES (GraspStudio PUBLIC Saba VirtualRobot ${QHULL_LIBRARIES})

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
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  ARCHIVE DESTINATION lib COMPONENT library
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

