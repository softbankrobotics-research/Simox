cmake_minimum_required (VERSION 2.6)
project (collada)

add_executable(collada ../collada.cpp ../inventor.cpp viewer.cpp ../pugixml/pugixml.cpp)

TARGET_INCLUDE_DIRECTORIES(collada PUBLIC "../pugixml" "/usr/include/qt4" "/usr/include/qt4/QT" "/usr/include/qt4/QtCore" "/usr/include/eigen3" )
target_link_libraries(collada Coin SoQt boost_system boost_timer)
target_compile_definitions(collada PUBLIC -DTIMER_DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
endif(DOXYGEN_FOUND)
