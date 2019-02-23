# Build and helper macros

function(VirtualRobotApplication name srcs incs)
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot)
endfunction()


function(VirtualRobotQtApplication name srcs incs mocFiles uiFiles)
    set(CMAKE_AUTOMOC "YES")
    set(CMAKE_AUTOUIC "YES")
    set(generatedUiFiles ${uiFiles})
    set(generatedMocFiles ${mocFiles})
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot)
endfunction()


function(VirtualRobotQtLibrary name srcs incs mocFiles uiFiles)
    set(generatedUiFiles ${uiFiles})
    set(generatedMocFiles ${mocFiles})
    set(CMAKE_AUTOMOC "YES")
    set(CMAKE_AUTOUIC "YES")

    ################################## LIBRARY ##############################
    ADD_LIBRARY(${name} SHARED ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot)
endfunction()


function(SimoxApplication name srcs incs)
    VirtualRobotApplication("${name}" "${srcs}" "${incs}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtApplication name srcs incs mocFiles uiFiles)
    VirtualRobotQtApplication("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")  
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtLibrary name srcs incs mocFiles uiFiles)
    VirtualRobotQtLibrary("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()
