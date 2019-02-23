PROJECT(ColCheckerPQP)

MESSAGE (STATUS " **** Configuring PQP")

# Specify headers and sources
FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/*.cpp)
FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/*.h)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${SRCS} ${INCS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR})

target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_PQP)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "3rd_Party")
IF(UNIX)
    # We are on Linux
    target_compile_options(${PROJECT_NAME} PUBLIC -fPIC)
ENDIF()

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
install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION bin COMPONENT bin
  LIBRARY DESTINATION lib COMPONENT shlib
  ARCHIVE DESTINATION lib COMPONENT library
  COMPONENT dev)
