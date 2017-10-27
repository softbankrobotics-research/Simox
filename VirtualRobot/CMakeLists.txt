PROJECT ( simox-virtualrobot )

MESSAGE (STATUS "\n***** CONFIGURING project simox-virtualrobot *****")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

if(NOT ("${CMAKE_VERSION}" VERSION_LESS 2.8.12))
    cmake_policy(SET CMP0022 OLD) # avoid INTERFACE_LINK_LIBRARIES warnings
ENDIF()
if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW) # qtmain add lib dependency (no effect in our case)
endif()

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")


if (Simox_USE_NLOPT)
    FIND_PACKAGE (NLOPT)
    if (NLOPT_FOUND)
        ADD_DEFINITIONS(-DUSE_NLOPT)
    endif()
endif()

########################### TESTING #####################################
MACRO(ADD_VR_TEST TEST_NAME)
    INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
    INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/..")
    ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})
    if (NOT Boost_USE_STATIC_LIBS)
        ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
    endif (NOT Boost_USE_STATIC_LIBS)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${TEST_NAME} simox-virtualrobot ${Simox_EXTERNAL_LIBRARIES} ${Boost_TEST_LIB})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "VirtualRobot Tests")
    ADD_TEST(NAME VirtualRobot_${TEST_NAME}
        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO()

#################################### FILES ##################################
SET(SOURCES
CollisionDetection/CollisionChecker.cpp
CollisionDetection/CollisionModel.cpp
CollisionDetection/CDManager.cpp
EndEffector/EndEffector.cpp
EndEffector/EndEffectorActor.cpp
Visualization/Visualization.cpp
Visualization/VisualizationNode.cpp
Visualization/ColorMap.cpp
Visualization/TriMeshModel.cpp
XML/BaseIO.cpp
XML/ModelIO.cpp
XML/SceneIO.cpp
XML/ObjectIO.cpp
XML/FileIO.cpp
XML/EndEffectorIO.cpp
IK/IKSolver.cpp
IK/AdvancedIKSolver.cpp
IK/DifferentialIK.cpp
IK/GenericIKSolver.cpp
IK/CoMIK.cpp
IK/JacobiProvider.cpp
IK/PoseQualityMeasurement.cpp
IK/PoseQualityManipulability.cpp
IK/PoseQualityExtendedManipulability.cpp
IK/HierarchicalIK.cpp
IK/HierarchicalIKSolver.cpp
IK/JointLimitAvoidanceJacobi.cpp
IK/GazeIK.cpp
IK/StackedIK.cpp
IK/SupportPolygon.cpp
IK/FeetPosture.cpp
IK/ConstrainedIK.cpp
IK/ConstrainedHierarchicalIK.cpp
IK/ConstrainedStackedIK.cpp
IK/Constraint.cpp
IK/constraints/BalanceConstraint.cpp
IK/constraints/PoseConstraint.cpp
IK/constraints/PositionConstraint.cpp
IK/constraints/OrientationConstraint.cpp
IK/constraints/TSRConstraint.cpp
IK/constraints/ReferenceConfigurationConstraint.cpp
IK/constraints/JointLimitAvoidanceConstraint.cpp
IK/constraints/CoMConstraint.cpp
Workspace/WorkspaceDataArray.cpp
Workspace/WorkspaceRepresentation.cpp
Workspace/Reachability.cpp
Workspace/Manipulability.cpp
Workspace/WorkspaceGrid.cpp
#Workspace/ReachabilityInversion/InverseReachability.cpp
#Workspace/ReachabilityInversion/OrientedWorkspaceGrid.cpp
#Workspace/ReachabilityInversion/ReachabilityProcessor.cpp
#Workspace/ReachabilityInversion/RobotPlacementIK.cpp
#Workspace/ReachabilityInversion/RobotPlacementTrajectoryIK.cpp
TimeOptimalTrajectory/Path.cpp
TimeOptimalTrajectory/TimeOptimalTrajectory.cpp
Grasping/Grasp.cpp
Grasping/GraspSet.cpp
Grasping/BasicGraspQualityMeasure.cpp
Model/Model.cpp
Model/ModelSet.cpp
Model/ModelConfig.cpp
Model/ModelNodeSet.cpp
Model/LinkSet.cpp
Model/JointSet.cpp
Model/ModelFactory.cpp
Model/Obstacle.cpp
Model/Primitive.cpp
Model/ManipulationObject.cpp
Model/Frame.cpp
Model/Nodes/ModelNode.cpp
Model/Nodes/ModelLink.cpp
Model/Nodes/ModelJoint.cpp
Model/Nodes/ModelJointFixed.cpp
Model/Nodes/ModelJointPrismatic.cpp
Model/Nodes/ModelJointRevolute.cpp
Model/Nodes/Attachments/ModelNodeAttachment.cpp
Model/Nodes/Attachments/ModelNodeAttachmentFactory.cpp
Model/Nodes/Attachments/ModelStructure.cpp
Model/Nodes/Attachments/ModelStructureFactory.cpp
Model/Nodes/Attachments/Sensor.cpp
Model/Nodes/Attachments/PositionSensor.cpp
Model/Nodes/Attachments/PositionSensorFactory.cpp
Model/Nodes/Attachments/ForceTorqueSensor.cpp
Model/Nodes/Attachments/ForceTorqueSensorFactory.cpp
Tools/MathTools.cpp
Tools/Orientation.cpp
Tools/BoundingBox.cpp
Tools/RuntimeEnvironment.cpp
Tools/LinkedCoordinate.cpp
Tools/SphereApproximator.cpp
Compression/CompressionRLE.cpp
Compression/CompressionBZip2.cpp
Import/SimoxXMLFactory.cpp
Import/RobotImporterFactory.cpp
Import/MeshImport/STLReader.cpp
Trajectory.cpp
Scene.cpp
VirtualRobotException.cpp
VirtualRobot.cpp
Tools/Gravity.cpp
)

SET(INCLUDES
CollisionDetection/CollisionChecker.h
CollisionDetection/CollisionModel.h
CollisionDetection/CDManager.h
CollisionDetection/CollisionModelImplementation.h
CollisionDetection/CollisionCheckerImplementation.h
EndEffector/EndEffector.h
EndEffector/EndEffectorActor.h
Visualization/VisualizationFactory.h
Visualization/Visualization.h
Visualization/VisualizationNode.h
Visualization/ColorMap.h
Visualization/TriMeshModel.h
Visualization/TriangleFace.h
XML/BaseIO.h
XML/ModelIO.h
XML/SceneIO.h
XML/ObjectIO.h
XML/FileIO.h
IK/IKSolver.h
XML/EndEffectorIO.h
IK/AdvancedIKSolver.h
IK/DifferentialIK.h
IK/GenericIKSolver.h
IK/CoMIK.h
IK/JacobiProvider.h
IK/PoseQualityMeasurement.h
IK/PoseQualityManipulability.h
IK/PoseQualityExtendedManipulability.h
IK/HierarchicalIK.h
IK/HierarchicalIKSolver.h
IK/JointLimitAvoidanceJacobi.h
IK/GazeIK.h
IK/StackedIK.h
IK/SupportPolygon.h
IK/FeetPosture.h
IK/ConstrainedIK.h
IK/ConstrainedHierarchicalIK.h
IK/ConstrainedStackedIK.h
IK/Constraint.h
IK/constraints/BalanceConstraint.h
IK/constraints/PoseConstraint.h
IK/constraints/PositionConstraint.h
IK/constraints/OrientationConstraint.h
IK/constraints/TSRConstraint.h
IK/constraints/ReferenceConfigurationConstraint.h
IK/constraints/JointLimitAvoidanceConstraint.h
IK/constraints/CoMConstraint.h
Workspace/WorkspaceData.h
Workspace/WorkspaceDataArray.h
Workspace/WorkspaceRepresentation.h
Workspace/Reachability.h
Workspace/Manipulability.h
Workspace/VoxelTree6D.hpp
Workspace/VoxelTree6DElement.hpp
Workspace/VoxelTreeND.hpp
Workspace/VoxelTreeNDElement.hpp
Workspace/WorkspaceGrid.h
#Workspace/ReachabilityInversion/InverseReachability.h
#Workspace/ReachabilityInversion/OrientedWorkspaceGrid.h
#Workspace/ReachabilityInversion/ReachabilityProcessor.h
#Workspace/ReachabilityInversion/RobotPlacementIK.h
#Workspace/ReachabilityInversion/RobotPlacementTrajectoryIK.h
TimeOptimalTrajectory/Path.h
TimeOptimalTrajectory/TimeOptimalTrajectory.h
Grasping/Grasp.h
Grasping/GraspSet.h
Grasping/BasicGraspQualityMeasure.h
Model/Model.h
Model/ModelSet.h
Model/ModelConfig.h
Model/ModelNodeSet.h
Model/LinkSet.h
Model/JointSet.h
Model/ModelFactory.h
Model/Obstacle.h
Model/Primitive.h
Model/ManipulationObject.h
Model/Frame.h
Model/Nodes/ModelNode.h
Model/Nodes/ModelLink.h
Model/Nodes/ModelJoint.h
Model/Nodes/ModelJointFixed.h
Model/Nodes/ModelJointPrismatic.h
Model/Nodes/ModelJointRevolute.h
Model/Nodes/Attachments/ModelNodeAttachment.h
Model/Nodes/Attachments/ModelNodeAttachmentFactory.h
Model/Nodes/Attachments/Sensor.h
Model/Nodes/Attachments/PositionSensor.h
Model/Nodes/Attachments/PositionSensorFactory.h
Model/Nodes/Attachments/ForceTorqueSensor.h
Model/Nodes/Attachments/ForceTorqueSensorFactory.h
Model/Nodes/Attachments/ModelStructure.h
Model/Nodes/Attachments/ModelStructureFactory.h
Tools/AbstractFactoryMethod.h
Tools/ConditionedLock.h
Tools/MathTools.h
Tools/Orientation.h
Tools/BoundingBox.h
Tools/RuntimeEnvironment.h
Tools/SphereApproximator.h
DataStructures/nanoflann.hpp
DataStructures/KdTreePointCloud.h
Compression/CompressionRLE.h
Compression/CompressionBZip2.h
Import/SimoxXMLFactory.h
Import/RobotImporterFactory.h
Import/MeshImport/STLReader.h
VirtualRobotException.h
VirtualRobotImportExport.h
VirtualRobotTest.h
Trajectory.h
Scene.h
VirtualRobot.h
Tools/Gravity.h

)

if (Simox_USE_NLOPT AND NLOPT_FOUND)
    SET(SOURCES ${SOURCES}
        IK/ConstrainedOptimizationIK.cpp
    )

    SET(INCLUDES ${INCLUDES}
        IK/ConstrainedOptimizationIK.h
    )
endif ()

if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)

    SET(SOURCES
    ${SOURCES}
    Visualization/CoinVisualization/CoinVisualizationFactory.cpp
    Visualization/CoinVisualization/CoinVisualizationNode.cpp
    Visualization/CoinVisualization/CoinVisualization.cpp
    )

    SET(INCLUDES
    ${INCLUDES}
    Visualization/CoinVisualization/CoinVisualizationFactory.h
    Visualization/CoinVisualization/CoinVisualizationNode.h
    Visualization/CoinVisualization/CoinVisualization.h
    )

    if(BUILD_TESTING)
        ADD_SUBDIRECTORY(Visualization/CoinVisualization/tests)
    endif()
endif ()

FIND_PACKAGE(urdfdom REQUIRED)
#check for URDF
if (Simox_USE_URDF)
    include(SearchForTinyXML)
    FIND_PACKAGE(urdfdom REQUIRED)
    #FIND_PACKAGE(PkgConfig)
    #PKG_CHECK_MODULES(URDFDOM REQUIRED urdfdom>=0.2.9)
    #PKG_CHECK_MODULES(URDFDOM_HEADERS REQUIRED urdfdom_headers>=0.2.3)

    if (tinyxml_library AND urdfdom_FOUND)
        MESSAGE(STATUS " ** Including URDF importer")

        include_directories(${tinyxml_include_dirs})
        link_directories(${tinyxml_library_dirs})
        add_definitions(${tinyxml_cflags})
        add_definitions(" -DTIXML_USE_STL ")

        INCLUDE_DIRECTORIES (
            ${urdfdom_INCLUDE_DIRS}
            ${urdfdom_PARSER_INCLUDE_DIRS}
            ${urdfdom_HEADERS_INCLUDE_DIRS}
        )

        SET(SOURCES
          ${SOURCES}
            Import/URDF/SimoxURDFFactory.cpp
            )
        SET(INCLUDES
          ${INCLUDES}
            Import/URDF/SimoxURDFFactory.h
            )

        SET(Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${urdfdom_LIBRARIES})
    else()
        MESSAGE(STATUS " *** urdfDom or TinyXML not found: skipping URDF importer...")
    endif()
endif()

# include collision detection
INCLUDE (CollisionDetection/CMakeLists.txt)

if(BUILD_TESTING)
    # include unit tests
    ADD_SUBDIRECTORY(tests/)
    ADD_SUBDIRECTORY(Visualization/tests)
    ADD_SUBDIRECTORY(Workspace/tests)
    ADD_SUBDIRECTORY(Compression/tests)
endif()

if(Simox_BUILD_EXAMPLES)
    # include examples
    ADD_SUBDIRECTORY(examples/)
endif()


ADD_LIBRARY (${PROJECT_NAME} SHARED ${SOURCES} ${INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ColCheckerPQP ${Simox_EXTERNAL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

#set_target_properties(${PROJECT_NAME} PROPERTIES
#  PUBLIC_HEADER "${INCLUDES}")

install(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
    COMPONENT dev)
    #  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/VirtualRobot"
 
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} DESTINATION "${Simox_INSTALL_HEADER_DIR}"
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

#INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}
#    COMPONENT ApplicationData
#        FILES_MATCHING
#        PATTERN "*.xml"
#        PATTERN "*.XML"
#        PATTERN "*.iv"
#        PATTERN "*.vrml"
#        PATTERN "*.wrl"
#        PATTERN "*.stl"
#        PATTERN "*.urdf"
#        PATTERN "*.dae"
#        PATTERN "*.png"
#        PATTERN "*.jpg"
#        PATTERN "*.bin"
#        PATTERN ".svn" EXCLUDE
#        PATTERN ".git" EXCLUDE
#        )
   
MESSAGE (STATUS "***** Finished CONFIGURING project simox-virtualrobot *****\n")
