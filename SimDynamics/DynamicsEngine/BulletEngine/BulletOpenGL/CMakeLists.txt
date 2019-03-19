# This is copied from the bullet sources since the openGL support library is usually not included in any linux packages.

PROJECT(BulletOpenGLSupport)

if (SimDynamics_USE_BULLET AND OPENGL_FOUND)

SET (SimDynamics_BULLET_OpenGL TRUE PARENT_SCOPE)

MESSAGE (STATUS " **** Configuring BulletOpenGLSupport")

# Specify headers and sources
FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/*.cpp)
FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/*.h)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${SRCS} ${INCS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${BULLET_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${GLUT_INCLUDE_DIR})
if(EXISTS "${BULLET_INCLUDE_DIRS}/bullet")
    target_include_directories(${PROJECT_NAME} PUBLIC ${BULLET_INCLUDE_DIRS}/bullet)
endif()

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "3rd_Party")
IF(UNIX)
	# We are on Linux
	ADD_DEFINITIONS(-fPIC)
ENDIF(UNIX)

MESSAGE(STATUS " ** ${PROJECT_NAME}  will be placed into " ${Simox_LIB_DIR})


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
#  PUBLIC_HEADER "${INCS}")

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  ARCHIVE DESTINATION lib COMPONENT library
  COMPONENT dev)

else()
MESSAGE (STATUS " **** Not compiling BulletOpenGL support")
endif()
