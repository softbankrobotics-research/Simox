PROJECT ( SimDynamics )

MESSAGE (STATUS "***** CONFIGURING Simox project SimDynamics *****")

add_library(SimDynamics SHARED SimDynamics.h) # sources will be added later

target_include_directories(SimDynamics PUBLIC
    $<BUILD_INTERFACE:${Simox_SOURCE_DIR}>
    $<BUILD_INTERFACE:${Simox_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

GET_FILENAME_COMPONENT (CurrentSimDynamicsPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(SimDynamics_DIR ${CurrentSimDynamicsPath})


############################# SETUP PHYSICS ENGINE #############################
SET (SimDynamics_DYNAMICSENGINE FALSE)

if (SimDynamics_USE_BULLET)

  IF(NOT "$ENV{BULLET_ROOT}" STREQUAL "")
    MESSAGE (STATUS " ** USING BULLET_ROOT-PATH from environment variable BULLET_ROOT: $ENV{BULLET_ROOT}")
    file(TO_CMAKE_PATH "$ENV{BULLET_ROOT}" BULLET_ROOT)
  ENDIF()
	#SET (SimDynamics_BULLET_ROOT ${Simox_BULLET_ROOT} CACHE PATH "Path to Bullet used by SimDynamics")
  SET (BULLET_ROOT ${BULLET_ROOT} CACHE PATH "Bullet Path")
  Find_Package(Bullet)
  if (BULLET_FOUND)
    MESSAGE (STATUS " ** Found Bullet at ${BULLET_INCLUDE_DIRS}")

    target_include_directories(SimDynamics PUBLIC ${BULLET_INCLUDE_DIRS})
    if(EXISTS "${BULLET_INCLUDE_DIRS}/bullet")
        target_include_directories(SimDynamics PUBLIC ${BULLET_INCLUDE_DIRS}/bullet)
    endif()

    IF (WIN32)
        OPTION(SimDynamics_USE_BULLET_USE_GLUT "Use Glut"	OFF)
    else()
        OPTION(SimDynamics_USE_BULLET_USE_GLUT "Use Glut"	ON)
    endif()

    include(BulletCheckPrecision)
    message(STATUS "BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS}")
    BulletCheckPrecision()
    target_link_libraries(SimDynamics PUBLIC ${BULLET_LIBRARIES})
    SET (SimDynamics_DYNAMICSENGINE TRUE)
      
    FIND_PACKAGE(OpenGL)
    IF (OPENGL_FOUND)
        MESSAGE (STATUS " ** OPENGL FOUND lib:${OPENGL_gl_LIBRARY}")
        #check if there was a distuingish between optimized and debug libs:
        IF (BULLET_LIBRARIES MATCHES "optimized;")
            target_link_libraries(SimDynamics PUBLIC optimized ${OPENGL_gl_LIBRARY} debug ${OPENGL_gl_LIBRARY})
        ELSE()
            target_link_libraries(SimDynamics PUBLIC  ${OPENGL_gl_LIBRARY})
        ENDIF()
    ENDIF()
    IF (OPENGL_GLU_FOUND)
        MESSAGE (STATUS " ** OPENGL_GLU FOUND lib:${OPENGL_glu_LIBRARY}")
        #check if there was a distuingish between optimized and debug libs:
        IF (BULLET_LIBRARIES MATCHES "optimized;")
            target_link_libraries(SimDynamics PUBLIC optimized ${OPENGL_glu_LIBRARY} debug ${OPENGL_glu_LIBRARY})
        ELSE()
            target_link_libraries(SimDynamics PUBLIC ${OPENGL_glu_LIBRARY})
        ENDIF()
    ENDIF()

    IF (SimDynamics_USE_BULLET_USE_GLUT)
        FIND_PACKAGE(GLUT)
        IF (GLUT_FOUND)
            MESSAGE(STATUS " ** GLUT FOUND: ${GLUT_glut_LIBRARY}")
            #check if there was a distuingish between optimized and debug libs:
            IF (BULLET_LIBRARIES MATCHES "optimized;")
                target_link_libraries(SimDynamics PUBLIC optimized ${GLUT_glut_LIBRARY} debug ${GLUT_glut_LIBRARY})
            ELSE()
                target_link_libraries(SimDynamics PUBLIC ${GLUT_glut_LIBRARY} )
            ENDIF()

            IF (WIN32)
                target_include_directories(SimDynamics PUBLIC ${GLUT_INCLUDE_DIR})
            ELSE ()
                # This is the lines for linux.  This should always work if everything is installed and working fine.
                target_include_directories(SimDynamics PUBLIC "/usr/include;/usr/local/include;${GLUT_INCLUDE_DIR}")
            ENDIF ()
        ELSE ()
            SET( GLUT_glut_LIBRARY "" CACHE PATH "Glut library." )
        ENDIF ()
    ENDIF()

    ADD_SUBDIRECTORY(DynamicsEngine/BulletEngine/BulletOpenGL)

else()
    MESSAGE (STATUS " ** Could not find Bullet")
endif()
    
else()
        MESSAGE ("No Physics engine selected...")   	    
endif()




#######################################################################
# Setup for testing
#######################################################################

MACRO(ADD_SIMDYNAMICS_TEST TEST_NAME)
  ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
  if (NOT Boost_USE_STATIC_LIBS)
      target_compile_definitions(${TEST_NAME} PRIVATE -DBOOST_TEST_DYN_LINK)
  endif ()
  TARGET_LINK_LIBRARIES(${TEST_NAME} PRIVATE VirtualRobot Saba GraspStudio SimDynamics Boost::unit_test_framework)
  SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
  SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "SimDynamics Tests")
  ADD_TEST(NAME SimDynamics_${TEST_NAME}
          COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_SIMDYNAMICS_TEST)


if (SimDynamics_DYNAMICSENGINE)

  SET(SOURCES
    DynamicsWorld.cpp
    DynamicsEngine/DynamicsObject.cpp
    DynamicsEngine/DynamicsEngine.cpp
    DynamicsEngine/DynamicsRobot.cpp
    DynamicsEngine/DynamicsUtils.cpp
  )
    
  SET(INCLUDES
    DynamicsWorld.h
    SimDynamics.h
    DynamicsEngine/DynamicsEngineFactory.h
    DynamicsEngine/DynamicsObject.h
    DynamicsEngine/DynamicsEngine.h
    DynamicsEngine/DynamicsRobot.h
    DynamicsEngine/DynamicsUtils.h
  )
   
  if (SimDynamics_USE_BULLET)
  	SET(SOURCES
    	${SOURCES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.cpp
    	DynamicsEngine/BulletEngine/BulletEngine.cpp
    	DynamicsEngine/BulletEngine/BulletObject.cpp
    	DynamicsEngine/BulletEngine/BulletRobot.cpp
    	DynamicsEngine/BulletEngine/BulletRobotLogger.cpp
    	DynamicsEngine/BulletEngine/SimoxMotionState.cpp
     	DynamicsEngine/BulletEngine/SimoxCollisionDispatcher.cpp
  	)
  
  	SET(INCLUDES
    	${INCLUDES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.h
    	DynamicsEngine/BulletEngine/BulletEngine.h
    	DynamicsEngine/BulletEngine/BulletObject.h
    	DynamicsEngine/BulletEngine/BulletRobot.h
    	DynamicsEngine/BulletEngine/BulletRobotLogger.h
    	DynamicsEngine/BulletEngine/SimoxMotionState.h
     	DynamicsEngine/BulletEngine/SimoxCollisionDispatcher.h
  	)
  	
  	if (SimDynamics_BULLET_OpenGL)
      	SET(SOURCES
        	${SOURCES}
			DynamicsEngine/BulletEngine/BulletOpenGLViewer.cpp
			)

      	SET(INCLUDES
        	${INCLUDES}
			DynamicsEngine/BulletEngine/BulletOpenGLViewer.h
			)

      else()
          MESSAGE (STATUS " ** Bullet's OpenGLSupport library not found: Skipping BulletOpenGLViewer")
      endif()
  
  if (Simox_USE_COIN_VISUALIZATION AND SOQT_FOUND)
      	SET(SOURCES
        	${SOURCES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.cpp
      	)
      
      	SET(INCLUDES
        	${INCLUDES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.h
      	)
      else()
          MESSAGE (STATUS "SOQT_FOUND                  : ${SOQT_FOUND}")
          MESSAGE (STATUS "Simox_USE_COIN_VISUALIZATION: ${Simox_USE_COIN_VISUALIZATION}")
          MESSAGE (STATUS " ** Qt/Coin3D disabled: Skipping BulletCoinQtViewer")
      endif()
  endif ()

  set_target_properties(SimDynamics PROPERTIES SOURCES "${SOURCES};${INCLUDES}")

  if (SimDynamics_BULLET_OpenGL)
    TARGET_LINK_LIBRARIES (SimDynamics PUBLIC BulletOpenGLSupport)
	SET (SimDynamics_BULLET_OpenGL TRUE PARENT_SCOPE) # inform parent scope about BulletOpenGLSupport
  ENDIF(SimDynamics_BULLET_OpenGL)
  TARGET_LINK_LIBRARIES (SimDynamics PUBLIC VirtualRobot)

  if (SimDynamics_USE_BULLET AND NOT BULLET_USE_SINGLE_PRECISION )
    target_compile_definitions(SimDynamics PUBLIC -DBT_USE_DOUBLE_PRECISION)
  endif()
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
  if (SimDynamics_USE_BULLET)
      ADD_SUBDIRECTORY(DynamicsEngine/BulletEngine/tests/)
  endif()
  
    
    
  #######################################################################################
  ############################ Setup for installation ###################################
  #######################################################################################
  
  
  INSTALL(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION bin COMPONENT bin
    LIBRARY DESTINATION lib COMPONENT shlib
    ARCHIVE DESTINATION lib COMPONENT library
    COMPONENT dev)
  
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
  
else()
    MESSAGE ("Not building SimDynamics, physics engine not found...")
endif()

MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimDynamics *****")
