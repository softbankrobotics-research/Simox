PROJECT ( VirtualRobot )

MESSAGE (STATUS "\n***** CONFIGURING Simox project VirtualRobot *****")

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

if (Simox_USE_RBDL)
    FIND_PACKAGE (RBDL)
endif()

if (Simox_USE_NLOPT)
    FIND_PACKAGE (NLOPT)
    if (NLOPT_FOUND)
        ADD_DEFINITIONS(-DUSE_NLOPT)
    endif()
endif()

########################### TESTING #####################################
MACRO(ADD_VR_TEST TEST_NAME)
    INCLUDE_DIRECTORIES(SYSTEM ${Simox_EXTERNAL_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
    INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/..")
    ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})
    if (NOT Boost_USE_STATIC_LIBS)
        ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
    endif (NOT Boost_USE_STATIC_LIBS)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot ${Simox_EXTERNAL_LIBRARIES} ${Boost_TEST_LIB})
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
Nodes/RobotNode.cpp
Nodes/RobotNodePrismatic.cpp
Nodes/RobotNodePrismaticFactory.cpp
Nodes/RobotNodeRevolute.cpp
Nodes/RobotNodeRevoluteFactory.cpp
Nodes/RobotNodeFixed.cpp
Nodes/RobotNodeFixedFactory.cpp
Nodes/RobotNodeActuator.cpp
Nodes/Sensor.cpp
Nodes/PositionSensor.cpp
Nodes/ForceTorqueSensor.cpp
Nodes/ContactSensor.cpp
Nodes/CameraSensor.cpp
Nodes/PositionSensorFactory.cpp
Nodes/ForceTorqueSensorFactory.cpp
Nodes/ContactSensorFactory.cpp
Nodes/CameraSensorFactory.cpp
Visualization/Visualization.cpp
Visualization/VisualizationNode.cpp
Visualization/ColorMap.cpp
Visualization/TriMeshModel.cpp
Visualization/TriMeshUtils.cpp
XML/BaseIO.cpp
XML/RobotIO.cpp
XML/SceneIO.cpp
XML/ObjectIO.cpp
XML/FileIO.cpp
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
IK/constraints/CollisionCheckConstraint.cpp
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
MathTools.cpp
Robot.cpp
RobotConfig.cpp
RobotNodeSet.cpp
Trajectory.cpp
KinematicChain.cpp
RobotFactory.cpp
SceneObject.cpp
SceneObjectSet.cpp
Scene.cpp
Obstacle.cpp
Primitive.cpp
VirtualRobotException.cpp
ManipulationObject.cpp
BoundingBox.cpp
RuntimeEnvironment.cpp
Random.cpp
LinkedCoordinate.cpp
SphereApproximator.cpp
VirtualRobot.cpp
Compression/CompressionRLE.cpp
Compression/CompressionBZip2.cpp
Import/SimoxXMLFactory.cpp
Import/RobotImporterFactory.cpp
Import/MeshImport/STLReader.cpp
Tools/Gravity.cpp
Tools/tinyxml2.cpp
math/Contact.cpp
math/ContactList.cpp
math/Line.cpp
math/LineStrip.cpp
math/Bezier.cpp
math/LinearContinuedBezier.cpp
math/LineR2.cpp
math/AbstractFunctionR1R2.cpp
math/AbstractFunctionR1R3.cpp
math/AbstractFunctionR1R6.cpp
math/AbstractFunctionR2R3.cpp
math/CompositeFunctionR1R6.cpp
math/TransformedFunctionR1R3.cpp
math/TransformedFunctionR2R3.cpp
math/LinearInterpolatedOrientation.cpp
math/FitPlane.cpp
math/Plane.cpp
math/Triangle.cpp
math/ImplicitPlane.cpp
math/GaussianObjectModel.cpp
math/GaussianObjectModelNormals.cpp
math/ImplicitObjectModel.cpp
math/Kernels.cpp
math/GaussianImplicitSurface3D.cpp
math/GaussianImplicitSurface3DNormals.cpp
math/GaussianImplicitSurface3DCombined.cpp
math/DataR3R1.cpp
math/DataR3R2.cpp
math/Helpers.cpp
math/Index3.cpp
math/MarchingCubes.cpp
math/Primitive.cpp
math/WeightedAverage.cpp
math/Grid3D.cpp
math/GridCacheFloat3.cpp
)

SET(INCLUDES
CollisionDetection/CollisionChecker.h
CollisionDetection/CollisionModel.h
CollisionDetection/CDManager.h
CollisionDetection/CollisionModelImplementation.h
CollisionDetection/CollisionCheckerImplementation.h
EndEffector/EndEffector.h
EndEffector/EndEffectorActor.h
Nodes/RobotNode.h
Nodes/RobotNodeFactory.h
Nodes/RobotNodePrismatic.h
Nodes/RobotNodePrismaticFactory.h
Nodes/RobotNodeRevolute.h
Nodes/RobotNodeRevoluteFactory.h
Nodes/RobotNodeFixed.h
Nodes/RobotNodeFixedFactory.h
Nodes/RobotNodeActuator.h
Nodes/ConditionedLock.h
Nodes/Sensor.h
Nodes/PositionSensor.h
Nodes/ForceTorqueSensor.h
Nodes/CameraSensor.h
Nodes/SensorFactory.h
Nodes/PositionSensorFactory.h
Nodes/ForceTorqueSensorFactory.h
Nodes/CameraSensorFactory.h
Transformation/DHParameter.h
Visualization/VisualizationFactory.h
Visualization/Visualization.h
Visualization/VisualizationNode.h
Visualization/ColorMap.h
Visualization/TriMeshModel.h
Visualization/TriMeshUtils.h
XML/BaseIO.h
XML/RobotIO.h
XML/SceneIO.h
XML/ObjectIO.h
XML/FileIO.h
IK/IKSolver.h
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
IK/constraints/CollisionCheckConstraint.h
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
AbstractFactoryMethod.h
VirtualRobot.h
MathTools.h
Robot.h
RobotConfig.h
RobotNodeSet.h
Trajectory.h
KinematicChain.h
RobotFactory.h
SceneObject.h
SceneObjectSet.h
Scene.h
Obstacle.h
Primitive.h
VirtualRobotException.h
VirtualRobotImportExport.h
VirtualRobotTest.h
ManipulationObject.h
BoundingBox.h
RuntimeEnvironment.h
Random.h
DataStructures/nanoflann.hpp
DataStructures/KdTreePointCloud.h
Compression/CompressionRLE.h
Compression/CompressionBZip2.h
SphereApproximator.h
Import/SimoxXMLFactory.h
Import/RobotImporterFactory.h
Import/MeshImport/STLReader.h
Tools/Gravity.h
Tools/tinyxml2.h
math/MathForwardDefinitions.h
math/Contact.h
math/ContactList.h
math/Line.h
math/LineStrip.h
math/Bezier.h
math/LinearContinuedBezier.h
math/LineR2.h
math/AbstractFunctionR1R2.h
math/AbstractFunctionR1R3.h
math/AbstractFunctionR1R6.h
math/AbstractFunctionR1Ori.h
math/AbstractFunctionR2R3.h
math/AbstractFunctionR3R1.h
math/TransformedFunctionR1R3.h
math/TransformedFunctionR2R3.h
math/SimpleAbstractFunctionR1R3.h
math/SimpleAbstractFunctionR1R6.h
math/SimpleAbstractFunctionR1Ori.h
math/SimpleAbstractFunctionR2R3.h
math/SimpleAbstractFunctionR3R1.h
math/CompositeFunctionR1R6.h
math/LinearInterpolatedOrientation.h
math/FitPlane.h
math/Plane.h
math/Triangle.h
math/ImplicitPlane.h
math/ImplicitObjectModel.cpp
math/GaussianObjectModel.h
math/GaussianObjectModelNormals.h
math/Kernels.h
math/GaussianImplicitSurface3D.h
math/GaussianImplicitSurface3DNormals.h
math/GaussianImplicitSurface3DCombined.h
math/DataR3R1.h
math/DataR3R2.h
math/Helpers.h
math/Index3.h
math/Array3D.h
math/MarchingCubes.h
math/Primitive.h
math/Grid3D.h
math/GridCacheFloat3.h
math/WeightedAverage.h

)

if (Simox_USE_RBDL AND RBDL_FOUND)
    SET(SOURCES
        ${SOURCES}
        Dynamics/dynamics.cpp
    )

    SET(INCLUDES
        ${INCLUDES}
        Dynamics/dynamics.h
    )

    if(BUILD_TESTING)
        ADD_SUBDIRECTORY(Dynamics/tests)
    endif()
endif ()

if(BUILD_TESTING)
    ADD_SUBDIRECTORY(Visualization/tests)
    ADD_SUBDIRECTORY(Workspace/tests)
    ADD_SUBDIRECTORY(Compression/tests)
endif()

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
    Import/COLLADA-light/ColladaSimox.cpp
    Import/COLLADA-light/collada.cpp
    Import/COLLADA-light/inventor.cpp
    Import/COLLADA-light/pugixml/pugixml.cpp
    Import/SimoxCOLLADAFactory.cpp
#    Workspace/ReachabilityInversion/Visualization/InverseReachabilityCoinVisualization.cpp
    )

    SET(INCLUDES
    ${INCLUDES}
    Visualization/CoinVisualization/CoinVisualizationFactory.h
    Visualization/CoinVisualization/CoinVisualizationNode.h
    Visualization/CoinVisualization/CoinVisualization.h
    Import/COLLADA-light/ColladaSimox.h
    Import/COLLADA-light/collada.h
    Import/COLLADA-light/pugixml/pugixml.hpp
    Import/SimoxCOLLADAFactory.cpp
    Import/COLLADA-light/inventor.h
    Import/SimoxCOLLADAFactory.h
#    Workspace/ReachabilityInversion/Visualization/InverseReachabilityCoinVisualization.h
    )

    if(BUILD_TESTING)
        ADD_SUBDIRECTORY(Visualization/CoinVisualization/tests)
    endif()

elseif (Simox_VISUALIZATION AND Simox_USE_OPENSCENEGRAPH_VISUALIZATION)

    SET(SOURCES
    ${SOURCES}
    Visualization/OSGVisualization/OSGVisualizationFactory.cpp
    Visualization/OSGVisualization/OSGVisualizationNode.cpp
    Visualization/OSGVisualization/OSGVisualization.cpp
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.cpp
    )

    SET(INCLUDES
    ${INCLUDES}
    Visualization/OSGVisualization/OSGVisualizationFactory.h
    Visualization/OSGVisualization/OSGVisualizationNode.h
    Visualization/OSGVisualization/OSGVisualization.h
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
    )

    set(GUI_MOC_HDRS
        Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
    )
    if (Simox_USE_QT4)
        qt4_wrap_cpp(SOURCES ${GUI_MOC_HDRS})
        else()
        qt5_wrap_cpp(SOURCES ${GUI_MOC_HDRS})
    endif()

    if(BUILD_TESTING)
        ADD_SUBDIRECTORY(Visualization/OSGVisualization/tests)
    endif()
endif ()

#if (Simox_USE_COLLADA)
#
#    SET(SOURCES
#    ${SOURCES}
#    Import/SimoxCOLLADAFactory.cpp
#    COLLADA/ColladaIO.cpp
#    COLLADA/ColladaParser.cpp
#    )
#
#    SET(INCLUDES
#    ${INCLUDES}
#    Import/SimoxCOLLADAFactory.h
#    COLLADA/ColladaIO.h
#    COLLADA/ColladaIterators.h
#    COLLADA/ColladaParser.h
#    )
#endif ()


#check for URDF
if (Simox_USE_URDF)
    include(SearchForTinyXML)
    FIND_PACKAGE(urdfdom)
    #FIND_PACKAGE(PkgConfig)
    #PKG_CHECK_MODULES(URDFDOM REQUIRED urdfdom>=0.2.9)
    #PKG_CHECK_MODULES(URDFDOM_HEADERS REQUIRED urdfdom_headers>=0.2.3)

    if (tinyxml_library AND URDF_FOUND)
        MESSAGE(STATUS " ** Including URDF importer")

        include_directories(SYSTEM ${tinyxml_include_dirs})
        link_directories(${tinyxml_library_dirs})
        add_definitions(${tinyxml_cflags})
        add_definitions(" -DTIXML_USE_STL ")

        INCLUDE_DIRECTORIES (SYSTEM
            ${URDFDOM_INCLUDE_DIRS}
            ${URDFDOM_PARSER_INCLUDE_DIRS}
            ${URDFDOM_HEADERS_INCLUDE_DIRS}
        )

        SET(SOURCES
          ${SOURCES}
            Import/URDF/SimoxURDFFactory.cpp
            )
        SET(INCLUDES
          ${INCLUDES}
            Import/URDF/SimoxURDFFactory.h
            )

        SET(Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${URDFDOM_LIBRARIES})
    else()
        MESSAGE(STATUS " *** urdfDom or TinyXML not found: skipping URDF importer...")
    endif()
endif()

# include collision detection
INCLUDE (CollisionDetection/CMakeLists.txt)

if(BUILD_TESTING)
    # include unit tests
    ADD_SUBDIRECTORY(tests/)
    ADD_SUBDIRECTORY(Transformation/tests/)
endif()

if(Simox_BUILD_EXAMPLES)
    # include examples
    ADD_SUBDIRECTORY(examples/)
endif()

# -----------------------------------------------------------------------------------
# Adding the UI files to the library
if (Simox_USE_COIN_VISUALIZATION AND SOQT_FOUND)
    MESSAGE (STATUS " ** Adding GraspEditor gui")
    set(UI
        Grasping/GraspEditor/GraspEditor.ui
    )
    
  if (Simox_USE_QT4)
        QT4_WRAP_UI(UI_HEADERS ${UI})
        QT4_WRAP_CPP(MOC_SRC
            Grasping/GraspEditor/GraspEditorWindow.h
        )
    else()
        QT5_WRAP_UI(UI_HEADERS ${UI})
        QT5_WRAP_CPP(MOC_SRC
            Grasping/GraspEditor/GraspEditorWindow.h
        )
  endif()

    
    set(INCLUDES ${INCLUDES} ${UI_HEADERS})

    SET(SOURCES ${SOURCES} ${MOC_SRC} "Grasping/GraspEditor/GraspEditorWindow.cpp")
    SET(INCLUDES ${INCLUDES} "Grasping/GraspEditor/GraspEditorWindow.h")
endif()

# Finished adding UI files
#-----------------------------------------------------------------------------------

ADD_LIBRARY (VirtualRobot SHARED ${SOURCES} ${INCLUDES})
TARGET_LINK_LIBRARIES(VirtualRobot PUBLIC ColCheckerPQP ${Simox_EXTERNAL_LIBRARIES})
target_include_directories(VirtualRobot SYSTEM PUBLIC ${Simox_EXTERNAL_INCLUDE_DIRS})

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

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION "."
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
   
MESSAGE (STATUS "***** Finished CONFIGURING Simox project VirtualRobot *****\n")
