
# Install the FlightControlLibrary
set(FLIGHT_CONTROLLER_LIBRARIES src/FlightControlUtils.cpp
                                src/Robot.cpp
                                src/IQPCost.cpp
                                src/IQPConstraint.cpp
                                src/QPInput.cpp
                                src/TrajectoryManager.cpp
                                src/JetModel.cpp)
set(FLIGHT_CONTROLLER_LIBRARIES_HDR include/FlightControlUtils.h
                                include/Robot.h
                                include/IQPCost.h
                                include/IQPConstraint.h
                                include/QPInput.h
                                include/TrajectoryManager.h
                                include/JetModel.h)
                                
# print FLIGHT_CONTROLLER_LIBRARIES
message(STATUS "FLIGHT_CONTROLLER_LIBRARIES: ${FLIGHT_CONTROLLER_LIBRARIES}")
message(STATUS "FLIGHT_CONTROLLER_LIBRARIES_HDR: ${FLIGHT_CONTROLLER_LIBRARIES_HDR}")

add_library(${CPP_FLIGHT_LIBRARY_NAME}  ${FLIGHT_CONTROLLER_LIBRARIES_HDR} ${FLIGHT_CONTROLLER_LIBRARIES} )
set_target_properties(${CPP_FLIGHT_LIBRARY_NAME} PROPERTIES PUBLIC_HEADER "${FLIGHT_CONTROLLER_LIBRARIES_HDR}")
target_include_directories(${CPP_FLIGHT_LIBRARY_NAME} PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${CPP_FLIGHT_LIBRARY_NAME} PUBLIC ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} OsqpEigen::OsqpEigen BipedalLocomotion::ParametersHandlerYarpImplementation 
  BipedalLocomotion::ParametersHandler BipedalLocomotion::VectorsCollection ${MATIO_LIBRARIES} YARP::YARP_robotinterface)

install(TARGETS ${CPP_FLIGHT_LIBRARY_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                     COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${CPP_FLIGHT_LIBRARY_NAME}" COMPONENT dev)

# Include bindings
add_subdirectory(bindings)

