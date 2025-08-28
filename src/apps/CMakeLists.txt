include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/configFolder/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/configFolder/ConfigFolderPath.h" @ONLY)

set(EXECUTABLE_NAME mpc-simple-example)

add_executable(${EXECUTABLE_NAME} testSimpleExample.cpp)

target_link_libraries(${EXECUTABLE_NAME} PRIVATE linearMomentumBasedMPC ${CPP_FLIGHT_LIBRARY_NAME} BipedalLocomotion::ParametersHandlerYarpImplementation)

# install the executable
install(TARGETS ${EXECUTABLE_NAME}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        COMPONENT bin)
