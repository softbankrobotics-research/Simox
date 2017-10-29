# Try to find shared object of the urdf library

SET (urdfdom_FOUND FALSE)


FIND_PATH(urdfdom_INCLUDE_DIRS urdfdom_model/model.h "$ENV{urdfdomDIR}/include" "$ENV{urdfdom_DIR}/include" "/usr/include")
IF (NOT urdfdom_INCLUDE_DIRS)
    FIND_PATH(urdfdom_INCLUDE_DIRS urdf_model/model.h "$ENV{urdfdomDIR}/include" "$ENV{urdfdom_DIR}/include" "/usr/include")
endif()
FIND_PATH(urdfdom_PARSER_INCLUDE_DIRS urdf_parser/urdf_parser.h "$ENV{urdfdomDIR}/include" "$ENV{urdfdom_DIR}/include")
#Running the following (commented) command will result in loading liburdf.so, when the ros-indigo is installed (with the respective paths loaded). However this is not the right library to load. See https://github.com/ros/rosdistro/issues/4633
#FIND_LIBRARY(urdfdom_LIBRARIES urdf "$ENV{urdfdomDIR}/lib" "$ENV{urdfdom_DIR}/lib" "/usr/lib" "/usr/lib/x86_64-linux-gnu")
#IF (NOT urdfdom_LIBRARIES)
    FIND_LIBRARY(urdfdom_LIBRARIES urdfdom_model "$ENV{urdfdomDIR}/lib" "$ENV{urdfdom_DIR}/lib" "/usr/lib" "/usr/lib/x86_64-linux-gnu")
#endif()
message(STATUS "include: ${urdfdom_INCLUDE_DIRS}")
message(STATUS "parser include ${urdfdom_PARSER_INCLUDE_DIRS}")
message(STATUS "libs:  ${urdfdom_LIBRARIES}")

IF (urdfdom_INCLUDE_DIRS AND urdfdom_PARSER_INCLUDE_DIRS AND urdfdom_LIBRARIES)
    SET (urdfdom_FOUND TRUE)
ENDIF ()

IF (urdfdom_FOUND)
   IF (NOT urdfdom_FIND_QUIETLY)
      MESSAGE(STATUS " ** Found urdfdom: ${urdfdom_LIBRARIES}")
   ENDIF (NOT urdfdom_FIND_QUIETLY)
ELSE (urdfdom_FOUND)
   IF (urdfdom_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find urdfdom")
   ENDIF (urdfdom_FIND_REQUIRED)
ENDIF (urdfdom_FOUND)

MARK_AS_ADVANCED (
    urdfdom_INCLUDE_DIRS
  urdfdom_PARSER_INCLUDE_DIRS
    urdfdom_LIBRARIES
	)
	
