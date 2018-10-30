macro(BulletCheckPrecision)
    message(STATUS "Testing Bullet for use of double precision...")
    try_compile(
        _resultDouble
        ${PROJECT_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/CMakeModules/BulletCheckPrecision.cpp
        CMAKE_FLAGS
            "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
            "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
            "-DCMAKE_CXX_FLAGS:string=-std=c++11"
        COMPILE_DEFINITIONS
            "-DBT_USE_DOUBLE_PRECISION"
        OUTPUT_VARIABLE _buildOutDouble
    )
    if( _resultDouble )
        message(STATUS "Bullet double precision detected. Automatically defining BT_USE_DOUBLE_PRECISION")
        set(BULLET_USE_SINGLE_PRECISION OFF CACHE BOOL "" FORCE)
        add_definitions(-DBT_USE_DOUBLE_PRECISION)
    else()
        # Try it *without* -DBT_USE_DOUBLE_PRECISION to make sure it's single...
        try_compile(
            _resultSingle
            ${PROJECT_BINARY_DIR}
            ${CMAKE_SOURCE_DIR}/CMakeModules/BulletCheckPrecision.cpp
            CMAKE_FLAGS
                "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
                "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
            OUTPUT_VARIABLE _buildOutSingle
        )
        if( _resultSingle )
            message(STATUS "Bullet single precision detected. Not defining BT_USE_DOUBLE_PRECISION")
            set(BULLET_USE_SINGLE_PRECISION ON CACHE BOOL "" FORCE)
        else()
            message(ERROR "Unable to determine single or double precision.")
            message(STATUS "----------------------------------")
            message(STATUS "Build config for double precision:")
            message(STATUS "    CMAKE_FLAGS")
            message(STATUS "        -DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}")
            message(STATUS "        -DLINK_LIBRARIES:string=${BULLET_LIBRARIES}")
            message(STATUS "        -DCMAKE_CXX_FLAGS:string=-std=c++11")
            message(STATUS "    COMPILE_DEFINITIONS")
            message(STATUS "        -DBT_USE_DOUBLE_PRECISION")
            message(STATUS "Build output for double precision:")
            message(STATUS "${_buildOutDouble}")
            message(STATUS "----------------------------------")
            message(STATUS "Build config for single precision:")
            message(STATUS "    CMAKE_FLAGS")
            message(STATUS "        -DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}")
            message(STATUS "        -DLINK_LIBRARIES:string=${BULLET_LIBRARIES}")
            message(STATUS "Build output for single precision:")
            message(STATUS "${_buildOutSingle}")
            message(FATAL_ERROR "Unable to determine single or double precision.")
        endif()
    endif()
endmacro()
