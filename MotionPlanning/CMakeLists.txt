PROJECT ( Saba )

MESSAGE (STATUS "***** CONFIGURING Simox project Saba *****")

find_package(Boost COMPONENTS unit_test_framework REQUIRED)


########################### TESTING #####################################
MACRO(ADD_SABA_TEST TEST_NAME)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/..)
    if (NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${TEST_NAME} PRIVATE -DBOOST_TEST_DYN_LINK)
    endif ()
    TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot Saba Boost::unit_test_framework)
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "Saba Tests")
    ADD_TEST(NAME Saba_${TEST_NAME}
    	        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_SABA_TEST)

#################################### FILES ##################################
SET(SOURCES
CSpace/CSpace.cpp
CSpace/CSpaceSampled.cpp
CSpace/CSpaceNode.cpp
CSpace/CSpacePath.cpp
CSpace/CSpaceTree.cpp
CSpace/Sampler.cpp
CSpace/ConfigurationConstraint.cpp
Planner/MotionPlanner.cpp
Planner/Rrt.cpp
Planner/BiRrt.cpp
Planner/GraspIkRrt.cpp
Planner/GraspRrt.cpp
Planner/PlanningThread.cpp
Visualization/RrtWorkspaceVisualization.cpp
PostProcessing/PathProcessor.cpp
PostProcessing/ShortcutProcessor.cpp
PostProcessing/ElasticBandProcessor.cpp
PostProcessing/PathProcessingThread.cpp
ApproachDiscretization.cpp
)

SET(INCLUDES
Saba.h
CSpace/CSpace.h
CSpace/CSpaceSampled.h
CSpace/CSpaceNode.h
CSpace/CSpacePath.h
CSpace/CSpaceTree.h
CSpace/Sampler.h
CSpace/ConfigurationConstraint.h
Planner/MotionPlanner.h
Planner/Rrt.h
Planner/BiRrt.h
Planner/GraspIkRrt.h
Planner/GraspRrt.h
Planner/PlanningThread.h
Visualization/RrtWorkspaceVisualization.h
PostProcessing/PathProcessor.h
PostProcessing/ShortcutProcessor.h
PostProcessing/ElasticBandProcessor.h
PostProcessing/PathProcessingThread.h
ApproachDiscretization.h
)
#../VirtualRobot/definesVR.h


if (Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h
	)
endif ()

ADD_LIBRARY (Saba SHARED ${SOURCES} ${INCLUDES})
#MESSAGE(STATUS "VirtualRobot_ROBOT_LINK_LIBRARIES:" ${VirtualRobot_EXTERNAL_LIBRARIES})
TARGET_LINK_LIBRARIES (Saba PUBLIC VirtualRobot)

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

if(BUILD_TESTING)
    # include unit tests
    ADD_SUBDIRECTORY(tests/)
endif()

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
MESSAGE (STATUS "***** Finished CONFIGURING Simox project Saba *****\n")
