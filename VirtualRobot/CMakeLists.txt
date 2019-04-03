PROJECT ( VirtualRobot )

add_library(VirtualRobot SHARED VirtualRobot.h) # sources will be added later

target_include_directories(VirtualRobot PUBLIC
    $<BUILD_INTERFACE:${Simox_SOURCE_DIR}>
    $<BUILD_INTERFACE:${Simox_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(VirtualRobot PUBLIC ${Simox_OS_compile_options})

#######################################################################
# Setup dependencies
#######################################################################
### RBDL
if (Simox_USE_RBDL)
    FIND_PACKAGE (RBDL)
    if (RBDL_FOUND)
        MESSAGE(STATUS "RBDL found at: ${RBDL_INCLUDE_DIR}")
        target_include_directories(VirtualRobot PUBLIC ${RBDL_INCLUDE_DIR})
        target_link_libraries(VirtualRobot PUBLIC ${RBDL_LIBRARIES})
    else ()
        MESSAGE(STATUS "RBDL not found!")
    endif ()
endif()

#### NLOPT
if (Simox_USE_NLOPT)
    FIND_PACKAGE(NLOPT)
    if (NLOPT_FOUND)
        MESSAGE(STATUS "NLOPT found at: ${NLOPT_INCLUDE_DIRS}")
        target_include_directories(VirtualRobot PUBLIC ${NLOPT_INCLUDE_DIRS})
        target_link_libraries(VirtualRobot PUBLIC ${NLOPT_LIBRARIES})
    else ()
        MESSAGE(STATUS "NLOPT not found")
    endif ()
endif ()

#### Eigen
FIND_PACKAGE (Eigen3 REQUIRED)
target_include_directories(VirtualRobot SYSTEM PUBLIC ${Eigen3_INCLUDE_DIR})

#### BOOST
FIND_PACKAGE(Boost COMPONENTS filesystem system program_options thread REQUIRED)
MESSAGE (STATUS "Boost found at: ${Boost_INCLUDE_DIR}")

target_include_directories(VirtualRobot PUBLIC ${Boost_INCLUDE_DIR})
target_link_libraries(VirtualRobot PUBLIC 
        Boost::boost
        Boost::filesystem
        Boost::system
        Boost::program_options
        Boost::thread
)
# disable boost auto linking
if (Boost_USE_STATIC_LIBS)
    target_compile_definitions(VirtualRobot PUBLIC -DBOOST_ALL_NO_LIB -DBOOST_TEST_MAIN)
else (Boost_USE_STATIC_LIBS)
    # enable dynamic linking for specific boost libraries
    target_compile_definitions(VirtualRobot PUBLIC
        -DBOOST_ALL_NO_LIB
        -DBOOST_PROGRAM_OPTIONS_DYN_LINK
        -DBOOST_FILESYSTEM_DYN_LINK
        -DBOOST_SYSTEM_DYN_LINK
        -DBOOST_THREAD_DYN_LINK)
endif ()


##########################################
#### VISUALIZATION Coin3D+Qt+SoQt / OSG+Qt


if (Simox_USE_COIN_VISUALIZATION)
    MESSAGE(STATUS "Searching Coin3D, Qt and SoQt...")

    ##### Coin3D
    FIND_PACKAGE(Coin3D REQUIRED)
    if (COIN3D_FOUND)
        MESSAGE (STATUS "Found Coin3D: " ${COIN3D_INCLUDE_DIRS})
        set(Simox_VISUALIZATION  TRUE CACHE BOOL "")

        target_include_directories(VirtualRobot PUBLIC ${COIN3D_INCLUDE_DIRS})
        target_link_libraries(VirtualRobot PUBLIC ${COIN3D_LIBRARIES})
        target_compile_definitions(VirtualRobot PUBLIC -DCOIN_DLL)
    endif ()


    if ( Qt5_FOUND )
        MESSAGE (STATUS "Found Qt5: " ${Qt5Gui_INCLUDE_DIRS}  ${Qt5OpenGL_INCLUDE_DIRS} )        
        target_link_libraries(VirtualRobot PUBLIC Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets)
    else ( )
        MESSAGE (STATUS "Did not find Qt. Disabling Qt/SoQt support.")
    endif ( )

    if (Qt5_FOUND AND COIN3D_FOUND)
        #### SoQt
        # This will set SoQt_INCLUDE_DIRS and SoQt_LIBRARIES
        FIND_PACKAGE(SoQt)
        set(SOQT_FOUND  ${SOQT_FOUND} CACHE INTERNAL "")

        if (NOT SOQT_FOUND)
            MESSAGE (STATUS "Did not find SoQt. Disabling SoQt support.")
        else ()
            MESSAGE (STATUS "Found SoQt:" ${SoQt_INCLUDE_DIRS})
            MESSAGE (STATUS "Enabling Coin3D/Qt/SoQt support")

            set(Simox_VISUALIZATION  TRUE CACHE BOOL "")
            set(Simox_BUILD_EXAMPLES ON CACHE BOOL "Build example applications")

            target_include_directories(VirtualRobot PUBLIC ${SoQt_INCLUDE_DIRS})
            target_link_libraries(VirtualRobot PUBLIC ${SoQt_LIBRARIES})
            target_compile_definitions(VirtualRobot PUBLIC -DSOQT_DLL -DSIMOX_USE_SOQT)

            if (NOT UNIX)
                target_link_libraries(VirtualRobot PUBLIC opengl32)
          endif()
        endif()
    endif()
elseif (Simox_USE_OPENSCENEGRAPH_VISUALIZATION)
    MESSAGE(STATUS "Searching OSG and Qt...")
    FIND_PACKAGE(OpenSceneGraph REQUIRED osgViewer osgUtil osgDB osgGA)

    if (Qt5_FOUND AND OPENSCENEGRAPH_FOUND)
        MESSAGE (STATUS "Enabling OSG/Qt support")
        ### a little hack is needed here since osgQt is not supported in the FindOSG script
        MESSAGE(STATUS "OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
        LIST(GET OPENSCENEGRAPH_LIBRARIES 1 firstOsgLib)
        MESSAGE(STATUS "firstOsgLib: ${firstOsgLib}")
        GET_FILENAME_COMPONENT(osgLibPath ${firstOsgLib} PATH)
        MESSAGE(STATUS "osgLibPath: ${osgLibPath}")
        if (UNIX)
            list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/libosgQt.so)
        else()
            list(APPEND OPENSCENEGRAPH_LIBRARIES optimized)
            list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQt.lib)
            list(APPEND OPENSCENEGRAPH_LIBRARIES debug)
            list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQtd.lib)
        endif()
        MESSAGE(STATUS "OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
        set(Simox_VISUALIZATION  TRUE CACHE BOOL "")


        target_include_directories(VirtualRobot PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})
        target_link_libraries(VirtualRobot PUBLIC ${OPENSCENEGRAPH_LIBRARIES})
    endif()
else()
    MESSAGE(STATUS "Visualization disabled")
endif()

if (Simox_USE_COLLADA)
    MESSAGE(STATUS "Searching for Collada...")
    FIND_PACKAGE(COLLADA_DOM REQUIRED 2.4)

    IF(COLLADA_DOM_FOUND)
        MESSAGE (STATUS "Found Collada")
        MESSAGE (STATUS "* Collada COLLADA_DOM_ROOT_DIR : ${COLLADA_DOM_ROOT_DIR}")
        MESSAGE (STATUS "* Collada Include DIRS: ${COLLADA_DOM_INCLUDE_DIRS}")
        MESSAGE (STATUS "* Collada Libs: ${COLLADA_DOM_LIBRARIES}")
        MESSAGE (STATUS "* Collada COLLADA_DOM_LIBRARY_DIRS: ${COLLADA_DOM_LIBRARY_DIRS}")

        FIND_LIBRARY(COLLADA_LIBRARY ${COLLADA_DOM_LIBRARIES} ${COLLADA_DOM_LIBRARY_DIRS})
        MESSAGE (STATUS "Collada Full Collada lib: ${COLLADA_LIBRARY}")

        target_include_directories(VirtualRobot PUBLIC ${COLLADA_DOM_INCLUDE_DIRS})
        target_link_libraries(VirtualRobot PUBLIC ${COLLADA_LIBRARY})
        target_compile_definitions(VirtualRobot PUBLIC -DDOM_DYNAMIC )
    ENDIF()
endif()

##########################################################################################################
##########################################################################################################
##########################################################################################################

MESSAGE (STATUS "\n***** CONFIGURING Simox project VirtualRobot *****")

if (Simox_USE_RBDL)
    FIND_PACKAGE (RBDL)
endif()

if (Simox_USE_NLOPT)
    FIND_PACKAGE (NLOPT)
endif()

########################### TESTING #####################################
MACRO(ADD_VR_TEST TEST_NAME)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot Boost::unit_test_framework)
    target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/..)
    if (NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${TEST_NAME} PRIVATE -DBOOST_TEST_DYN_LINK)
    endif ()
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "VirtualRobot Tests")
    ADD_TEST(NAME VirtualRobot_${TEST_NAME}
        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO()

#################################### FILES ##################################
SET(SOURCES
    
    BoundingBox.cpp
    KinematicChain.cpp
    LinkedCoordinate.cpp
    ManipulationObject.cpp
    MathTools.cpp
    Obstacle.cpp
    Primitive.cpp
    Random.cpp
    Robot.cpp
    RobotConfig.cpp
    RobotFactory.cpp
    RobotNodeSet.cpp
    RuntimeEnvironment.cpp
    Scene.cpp
    SceneObject.cpp
    SceneObjectSet.cpp
    SphereApproximator.cpp
    Trajectory.cpp
    VirtualRobot.cpp
    VirtualRobotException.cpp
    
    CollisionDetection/CDManager.cpp
    CollisionDetection/CollisionChecker.cpp
    CollisionDetection/CollisionModel.cpp
    
    Compression/CompressionBZip2.cpp
    Compression/CompressionRLE.cpp
    
    EndEffector/EndEffector.cpp
    EndEffector/EndEffectorActor.cpp
    
    Grasping/BasicGraspQualityMeasure.cpp
    Grasping/Grasp.cpp
    Grasping/GraspSet.cpp
    
    IK/AdvancedIKSolver.cpp
    IK/CoMIK.cpp
    IK/ConstrainedHierarchicalIK.cpp
    IK/ConstrainedIK.cpp
    IK/ConstrainedStackedIK.cpp
    IK/Constraint.cpp
    IK/DifferentialIK.cpp
    IK/FeetPosture.cpp
    IK/GazeIK.cpp
    IK/GenericIKSolver.cpp
    IK/HierarchicalIK.cpp
    IK/HierarchicalIKSolver.cpp
    IK/IKSolver.cpp
    IK/JacobiProvider.cpp
    IK/JointLimitAvoidanceJacobi.cpp
    IK/PoseQualityExtendedManipulability.cpp
    IK/PoseQualityManipulability.cpp
    IK/PoseQualityMeasurement.cpp
    IK/StackedIK.cpp
    IK/SupportPolygon.cpp
    IK/constraints/BalanceConstraint.cpp
    IK/constraints/CoMConstraint.cpp
    IK/constraints/CollisionCheckConstraint.cpp
    IK/constraints/JointLimitAvoidanceConstraint.cpp
    IK/constraints/OrientationConstraint.cpp
    IK/constraints/PoseConstraint.cpp
    IK/constraints/PositionConstraint.cpp
    IK/constraints/ReferenceConfigurationConstraint.cpp
    IK/constraints/TSRConstraint.cpp
    
    Import/RobotImporterFactory.cpp
    Import/SimoxXMLFactory.cpp
    Import/MeshImport/STLReader.cpp
    
    math/AbstractFunctionR1R2.cpp
    math/AbstractFunctionR1R3.cpp
    math/AbstractFunctionR1R6.cpp
    math/AbstractFunctionR2R3.cpp
    math/Bezier.cpp
    math/CompositeFunctionR1R6.cpp
    math/Contact.cpp
    math/ContactList.cpp
    math/DataR3R1.cpp
    math/DataR3R2.cpp
    math/FitPlane.cpp
    math/GaussianImplicitSurface3D.cpp
    math/GaussianImplicitSurface3DNormals.cpp
    math/GaussianImplicitSurface3DCombined.cpp
    math/GaussianObjectModel.cpp
    math/GaussianObjectModelNormals.cpp
    math/Grid3D.cpp
    math/GridCacheFloat3.cpp
    math/Helpers.cpp
    math/ImplicitPlane.cpp
    math/ImplicitObjectModel.cpp
    math/Index3.cpp
    math/Kernels.cpp
    math/Line.cpp
    math/LinearContinuedBezier.cpp
    math/LinearInterpolatedOrientation.cpp
    math/LineR2.cpp
    math/LineStrip.cpp
    math/MarchingCubes.cpp
    math/TransformedFunctionR1R3.cpp
    math/TransformedFunctionR2R3.cpp
    math/Plane.cpp
    math/Primitive.cpp
    math/Triangle.cpp
    math/WeightedAverage.cpp
    math/statistics/measures.cpp
    math/statistics/BoxPlot.cpp
    math/statistics/Histogram.cpp
    
    MJCF/Document.cpp
    MJCF/elements/core/AnyElement.cpp
    MJCF/elements/core/Attribute.cpp
    MJCF/elements/core/Element.cpp
    MJCF/elements/core/exceptions.cpp
    MJCF/elements/core/mjcf_utils.cpp
    MJCF/elements/core/Visitor.cpp
    MJCF/elements/types/actuator.cpp
    MJCF/elements/types/asset.cpp
    MJCF/elements/types/body.cpp
    MJCF/elements/types/contact.cpp
    MJCF/elements/types/compiler.cpp
    MJCF/elements/types/custom.cpp
    MJCF/elements/types/default.cpp
    MJCF/elements/types/equality.cpp
    MJCF/elements/types/keyframe.cpp
    MJCF/elements/types/meta.cpp
    MJCF/elements/types/option.cpp
    MJCF/elements/types/sensor.cpp
    MJCF/elements/types/size.cpp
    MJCF/elements/types/statistic.cpp
    MJCF/elements/types/tendon.cpp
    MJCF/elements/types/visual.cpp
    MJCF/visitors/Collector.cpp
    
    Nodes/CameraSensor.cpp
    Nodes/CameraSensorFactory.cpp
    Nodes/ContactSensor.cpp
    Nodes/ContactSensorFactory.cpp
    Nodes/ForceTorqueSensor.cpp
    Nodes/ForceTorqueSensorFactory.cpp
    Nodes/PositionSensor.cpp
    Nodes/PositionSensorFactory.cpp
    Nodes/RobotNode.cpp
    Nodes/RobotNodeActuator.cpp
    Nodes/RobotNodeFixed.cpp
    Nodes/RobotNodeFixedFactory.cpp
    Nodes/RobotNodePrismatic.cpp
    Nodes/RobotNodePrismaticFactory.cpp
    Nodes/RobotNodeRevolute.cpp
    Nodes/RobotNodeRevoluteFactory.cpp
    Nodes/Sensor.cpp
    
    TimeOptimalTrajectory/Path.cpp
    TimeOptimalTrajectory/TimeOptimalTrajectory.cpp
    
    Tools/Gravity.cpp    
    
    Util/json/eigen_conversion.cpp
    Util/xml/tinyxml2.cpp
    
    Visualization/ColorMap.cpp
    Visualization/TriMeshModel.cpp
    Visualization/TriMeshUtils.cpp
    Visualization/Visualization.cpp
    Visualization/VisualizationNode.cpp
    
    Workspace/Manipulability.cpp
    Workspace/Reachability.cpp
    Workspace/WorkspaceDataArray.cpp
    Workspace/WorkspaceGrid.cpp
    Workspace/WorkspaceRepresentation.cpp
    #Workspace/ReachabilityInversion/InverseReachability.cpp
    #Workspace/ReachabilityInversion/OrientedWorkspaceGrid.cpp
    #Workspace/ReachabilityInversion/ReachabilityProcessor.cpp
    #Workspace/ReachabilityInversion/RobotPlacementIK.cpp
    #Workspace/ReachabilityInversion/RobotPlacementTrajectoryIK.cpp
    
    XML/BaseIO.cpp
    XML/FileIO.cpp
    XML/ObjectIO.cpp
    XML/RobotIO.cpp
    XML/SceneIO.cpp
    XML/mjcf/MasslessBodySanitizer.cpp
    XML/mjcf/MujocoIO.cpp
)

SET(INCLUDES
    
    AbstractFactoryMethod.h
    BoundingBox.h
    KinematicChain.h
    ManipulationObject.h
    MathTools.h
    Obstacle.h
    Primitive.h
    Random.h
    Robot.h
    RobotConfig.h
    RobotFactory.h
    RobotNodeSet.h
    RuntimeEnvironment.h
    Scene.h
    SceneObject.h
    SceneObjectSet.h
    SphereApproximator.h
    Trajectory.h
    VirtualRobot.h
    VirtualRobotException.h
    VirtualRobotImportExport.h
    VirtualRobotTest.h
    
    CollisionDetection/CollisionChecker.h
    CollisionDetection/CollisionModel.h
    CollisionDetection/CDManager.h
    CollisionDetection/CollisionModelImplementation.h
    CollisionDetection/CollisionCheckerImplementation.h
    
    Compression/CompressionBZip2.h
    Compression/CompressionRLE.h
    
    DataStructures/nanoflann.hpp
    DataStructures/KdTreePointCloud.h
    
    EndEffector/EndEffector.h
    EndEffector/EndEffectorActor.h
    
    Grasping/Grasp.h
    Grasping/GraspSet.h
    Grasping/BasicGraspQualityMeasure.h
    
    IK/AdvancedIKSolver.h
    IK/CoMIK.h
    IK/ConstrainedHierarchicalIK.h
    IK/ConstrainedIK.h
    IK/ConstrainedStackedIK.h
    IK/Constraint.h
    IK/DifferentialIK.h
    IK/FeetPosture.h
    IK/GazeIK.h
    IK/GenericIKSolver.h
    IK/HierarchicalIK.h
    IK/HierarchicalIKSolver.h
    IK/IKSolver.h
    IK/JacobiProvider.h
    IK/JointLimitAvoidanceJacobi.h
    IK/PoseQualityExtendedManipulability.h
    IK/PoseQualityManipulability.h
    IK/PoseQualityMeasurement.h
    IK/StackedIK.h
    IK/SupportPolygon.h
    IK/constraints/BalanceConstraint.h
    IK/constraints/CoMConstraint.h
    IK/constraints/CollisionCheckConstraint.h
    IK/constraints/JointLimitAvoidanceConstraint.h
    IK/constraints/OrientationConstraint.h
    IK/constraints/PoseConstraint.h
    IK/constraints/PositionConstraint.h
    IK/constraints/ReferenceConfigurationConstraint.h
    IK/constraints/TSRConstraint.h

    Import/RobotImporterFactory.h
    Import/SimoxXMLFactory.h
    Import/MeshImport/STLReader.h
    
    math/AbstractFunctionR1Ori.h
    math/AbstractFunctionR1R2.h
    math/AbstractFunctionR1R3.h
    math/AbstractFunctionR1R6.h
    math/AbstractFunctionR2R3.h
    math/AbstractFunctionR3R1.h
    math/Array3D.h
    math/Bezier.h
    math/ClampedNormalDistribution.hpp
    math/CompositeFunctionR1R6.h
    math/Contact.h
    math/ContactList.h
    math/DataR3R1.h
    math/DataR3R2.h
    math/FitPlane.h
    math/GaussianImplicitSurface3D.h
    math/GaussianImplicitSurface3DCombined.h
    math/GaussianImplicitSurface3DNormals.h
    math/GaussianObjectModel.h
    math/GaussianObjectModelNormals.h
    math/Grid3D.h
    math/GridCacheFloat3.h
    math/Helpers.h
    math/ImplicitObjectModel.cpp
    math/ImplicitPlane.h
    math/Index3.h
    math/Kernels.h
    math/Line.h
    math/LineR2.h
    math/LineStrip.h
    math/LineStripR1RM.h
    math/LinearContinuedBezier.h
    math/LinearInterpolatedOrientation.h
    math/MarchingCubes.h
    math/MathForwardDefinitions.h
    math/Plane.h
    math/Primitive.h
    math/SimpleAbstractFunctionR1Ori.h
    math/SimpleAbstractFunctionR1R3.h
    math/SimpleAbstractFunctionR1R6.h
    math/SimpleAbstractFunctionR2R3.h
    math/SimpleAbstractFunctionR3R1.h
    math/SimpleAbstractFunctionRNRM.h
    math/TransformedFunctionR1R3.h
    math/TransformedFunctionR2R3.h
    math/Triangle.h
    math/WeightedAverage.h
    math/statistics/measures.h
    math/statistics/BoxPlot.h
    math/statistics/Histogram.h
    
    MJCF/Document.h
    MJCF/elements.h
    MJCF/elements/has_member.hpp
    MJCF/elements/core/AnyElement.h
    MJCF/elements/core/Attribute.h
    MJCF/elements/core/const_aware_ptr.hpp
    MJCF/elements/core/Element.h
    MJCF/elements/core/exceptions.h
    MJCF/elements/core/mjcf_utils.h
    MJCF/elements/core/Visitor.h
    MJCF/elements/types/actuator.h
    MJCF/elements/types/asset.h
    MJCF/elements/types/body.h
    MJCF/elements/types/compiler.h
    MJCF/elements/types/contact.h
    MJCF/elements/types/custom.h
    MJCF/elements/types/default.h
    MJCF/elements/types/equality.h
    MJCF/elements/types/keyframe.h
    MJCF/elements/types/meta.h
    MJCF/elements/types/option.h
    MJCF/elements/types/sensor.h
    MJCF/elements/types/size.h
    MJCF/elements/types/statistic.h
    MJCF/elements/types/tendon.h
    MJCF/elements/types/visual.h
    MJCF/visitors.h
    MJCF/visitors/Collector.h
    
    Nodes/CameraSensor.h
    Nodes/CameraSensorFactory.h
    Nodes/ConditionedLock.h
    Nodes/ForceTorqueSensor.h
    Nodes/ForceTorqueSensorFactory.h
    Nodes/PositionSensor.h
    Nodes/PositionSensorFactory.h
    Nodes/RobotNode.h
    Nodes/RobotNodeActuator.h
    Nodes/RobotNodeFactory.h
    Nodes/RobotNodeFixed.h
    Nodes/RobotNodeFixedFactory.h
    Nodes/RobotNodePrismatic.h
    Nodes/RobotNodePrismaticFactory.h
    Nodes/RobotNodeRevolute.h
    Nodes/RobotNodeRevoluteFactory.h
    Nodes/Sensor.h
    Nodes/SensorFactory.h
    
    TimeOptimalTrajectory/Path.h
    TimeOptimalTrajectory/TimeOptimalTrajectory.h
    
    Tools/Gravity.h
    
    Transformation/DHParameter.h
    
    Util/json/eigen_conversion.h
    Util/json/eigen_conversion.hpp
    Util/json/json.hpp
    Util/xml/tinyxml2.h
    
    Visualization/ColorMap.h
    Visualization/TriMeshModel.h
    Visualization/TriMeshUtils.h
    Visualization/Visualization.h
    Visualization/VisualizationFactory.h
    Visualization/VisualizationNode.h
    
    Workspace/Manipulability.h
    Workspace/Reachability.h
    Workspace/VoxelTree6D.hpp
    Workspace/VoxelTree6DElement.hpp
    Workspace/VoxelTreeND.hpp
    Workspace/VoxelTreeNDElement.hpp
    Workspace/WorkspaceData.h
    Workspace/WorkspaceDataArray.h
    Workspace/WorkspaceGrid.h
    Workspace/WorkspaceRepresentation.h
    #Workspace/ReachabilityInversion/InverseReachability.h
    #Workspace/ReachabilityInversion/OrientedWorkspaceGrid.h
    #Workspace/ReachabilityInversion/ReachabilityProcessor.h
    #Workspace/ReachabilityInversion/RobotPlacementIK.h
    #Workspace/ReachabilityInversion/RobotPlacementTrajectoryIK.h
    
    XML/BaseIO.h
    XML/FileIO.h
    XML/ObjectIO.h
    XML/RobotIO.h
    XML/SceneIO.h
    XML/mjcf/MasslessBodySanitizer.h
    XML/mjcf/MujocoIO.h
    
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

MESSAGE (STATUS "Simox_VISUALIZATION                   : ${Simox_VISUALIZATION}")
MESSAGE (STATUS "Simox_USE_COIN_VISUALIZATION          : ${Simox_USE_COIN_VISUALIZATION}")
MESSAGE (STATUS "Simox_USE_OPENSCENEGRAPH_VISUALIZATION: ${Simox_USE_OPENSCENEGRAPH_VISUALIZATION}")
if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)    
    MESSAGE (STATUS "adding Coin3d visualzation")
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

    if(BUILD_TESTING)
        ADD_SUBDIRECTORY(Visualization/OSGVisualization/tests)
    endif()
endif ()

#check for URDF
if (Simox_USE_URDF)
    include(SearchForTinyXML)
    FIND_PACKAGE(urdfdom)

    if (tinyxml_library AND URDF_FOUND)
        MESSAGE(STATUS " ** Including URDF importer")

        SET(SOURCES
          ${SOURCES}
            Import/URDF/SimoxURDFFactory.cpp
            )
        SET(INCLUDES
          ${INCLUDES}
            Import/URDF/SimoxURDFFactory.h
            )

        target_link_libraries(VirtualRobot PUBLIC ${URDFDOM_LIBRARIES})
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
    

    SET(SOURCES ${SOURCES} Grasping/GraspEditor/GraspEditorWindow.cpp)
    SET(INCLUDES ${INCLUDES}
        Grasping/GraspEditor/GraspEditorWindow.h
        Grasping/GraspEditor/GraspEditor.ui
    )
endif()

# Finished adding UI files
#-----------------------------------------------------------------------------------
set_target_properties(VirtualRobot PROPERTIES SOURCES "${SOURCES};${INCLUDES}")

target_compile_definitions(VirtualRobot PUBLIC
    "-DVirtualRobot_SRC_DATA_PATH=\"${Simox_DATA_DIR}\""
    "-DSimox_DATA_PATH=\"${Simox_INSTALL_DATA_DIR}\""
)
if (Simox_USE_NLOPT AND NLOPT_FOUND)
    target_compile_definitions(VirtualRobot PUBLIC -DUSE_NLOPT)
endif()


if (Simox_USE_URDF AND tinyxml_library AND URDF_FOUND)
        target_include_directories(VirtualRobot SYSTEM PUBLIC ${tinyxml_include_dirs})
        target_compile_options(VirtualRobot PUBLIC ${tinyxml_library_dirs})
        target_compile_definitions(VirtualRobot PUBLIC ${tinyxml_cflags} -DTIXML_USE_STL)

        target_include_directories(VirtualRobot SYSTEM PUBLIC
            ${URDFDOM_INCLUDE_DIRS}
            ${URDFDOM_PARSER_INCLUDE_DIRS}
            ${URDFDOM_HEADERS_INCLUDE_DIRS}
        )
endif()

# against undefined reference to boost::filesystem::detail::copy_file
# source: https://stackoverflow.com/a/3500721
target_compile_definitions(${PROJECT_NAME} PRIVATE -DBOOST_NO_CXX11_SCOPED_ENUMS)


# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})




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
