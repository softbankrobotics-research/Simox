
ADD_VR_TEST( VirtualRobotExceptionTest )

ADD_VR_TEST( VirtualRobotRobotTest )

ADD_VR_TEST( VirtualRobotTransformationTest )

if (Simox_VISUALIZATION)
	ADD_VR_TEST( VirtualRobotCollisionTest )
endif()

ADD_VR_TEST( VirtualRobotJacobianTest )

ADD_VR_TEST( VirtualRobotLinkedCoordinateTest )

ADD_VR_TEST( VirtualRobotSceneTest )

ADD_VR_TEST( VirtualRobotConfigTest )

ADD_VR_TEST( VirtualRobotMathToolsTest )

ADD_VR_TEST( VirtualRobotThreadsafetyTest )
ADD_VR_TEST( VirtualRobotTrajectoryTest )
ADD_VR_TEST( VirtualRobotWorkSpaceTest )
ADD_VR_TEST( VirtualRobotWorkSpaceGridTest )
ADD_VR_TEST( VirtualRobotSensorTest )
ADD_VR_TEST( VirtualRobotIOTest )
ADD_VR_TEST( VirtualRobotGazeIKTest )
ADD_VR_TEST( VirtualRobotMeshImportTest )

ADD_VR_TEST( VirtualRobotTimeOptimalTrajectoryTest )

ADD_VR_TEST( VirtualRobotJsonEigenConversionTest )

ADD_VR_TEST( MathFitPlaneTest )
ADD_VR_TEST( MathGaussianImplicitSurface3DNormalsTest )
ADD_VR_TEST( MathGaussianImplicitSurface3DTest )
ADD_VR_TEST( MathHelpersTest )
