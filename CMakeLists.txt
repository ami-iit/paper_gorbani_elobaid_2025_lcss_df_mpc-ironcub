
cmake_minimum_required(VERSION 3.16)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

project(data-fused-mpc-ironcub VERSION 0.0.1)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_definitions(-D_USE_MATH_DEFINES)

file(TO_NATIVE_PATH "${PROJECT_SOURCE_DIR}" REPO_ROOT_DIR)

# Load dependencies
find_package(YARP 3.1 REQUIRED COMPONENTS robotinterface)
find_package(iDynTree REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(BipedalLocomotionFramework 0.19.0 REQUIRED)
find_package(QpSolversEigen REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            PRIVATE_DEPENDENCIES OsqpEigen BipedalLocomotionFramework)
