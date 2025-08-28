# Author: Gabriele Nava
# Date: Apr. 2023

# Check the Operative System
if(MSVC OR WIN32)
	set(EXPORT_VAR "set")
	set(PATH_SEPARATOR "\\")
	set(VARIABLE_START "%")
	set(VARIABLE_END "%")
	set(PATH_LIST_SEPARATOR ";")
	set(PLUGIN_FOLDER_NAME "bin")
	set(SETUP_SCRIPT_EXTENSION ".bat")
elseif(UNIX)
	set(EXPORT_VAR "export")
	set(PATH_SEPARATOR "/")
	set(VARIABLE_START "$")
	set(VARIABLE_END "")
	set(PATH_LIST_SEPARATOR ":")
	set(PLUGIN_FOLDER_NAME "lib")
	set(SETUP_SCRIPT_EXTENSION ".sh")
endif()

set(NEW_LINE "\n")
set(CPP_FLIGHT_LIBRARY_NAME FlightControlLibrary)

######################## GET PYTHON_INSTALL_DIR ########################

# Find Python
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Development Interpreter REQUIRED)

if(Python3_VERSION VERSION_GREATER_EQUAL 3.12)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import os;import sysconfig;relative_site_packages = sysconfig.get_path('purelib').replace(sysconfig.get_path('data'), '').lstrip(os.path.sep);print(relative_site_packages)"
                    OUTPUT_VARIABLE _PYTHON_INSTDIR)
else()
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                    OUTPUT_VARIABLE _PYTHON_INSTDIR)
endif()
string(STRIP ${_PYTHON_INSTDIR} PYTHON_INSTALL_DIR)
message(STATUS "Python install directory: ${PYTHON_INSTALL_DIR}")

######################## Flight Controller Library #############################

add_subdirectory(flight-controller)

######################## Applications #############################

add_subdirectory(apps)

############################### UPDATE PYTHONPATH ################################

set(SETUP_PYTHONPATH "${EXPORT_VAR} PYTHONPATH=${VARIABLE_START}PYTHONPATH${VARIABLE_END}${PATH_LIST_SEPARATOR}${CMAKE_INSTALL_PREFIX}${PATH_SEPARATOR}${PYTHON_INSTALL_DIR}
${EXPORT_VAR} PYTHONPATH=${VARIABLE_START}PYTHONPATH${VARIABLE_END}${PATH_LIST_SEPARATOR}${VARIABLE_START}PROJECT_ROOT_PATH${VARIABLE_END}${PATH_SEPARATOR}src${PATH_SEPARATOR}mujoco_lib")

############################## REQUIRED LIBRARIES ################################

set(SETUP_LD_LIBRARY_PATH "${EXPORT_VAR} LD_LIBRARY_PATH=${VARIABLE_START}LD_LIBRARY_PATH${VARIABLE_END}${PATH_LIST_SEPARATOR}${CMAKE_INSTALL_PREFIX}${PATH_SEPARATOR}${PLUGIN_FOLDER_NAME}")

################################################################################

set(SETUP_YARP_DATA_DIRS "${EXPORT_VAR} YARP_DATA_DIRS=${VARIABLE_START}YARP_DATA_DIRS${VARIABLE_END}${PATH_LIST_SEPARATOR}${VARIABLE_START}PROJECT_ROOT_PATH${VARIABLE_END}${PATH_SEPARATOR}src${PATH_SEPARATOR}trajectories")

set(SETUP_PATH "${EXPORT_VAR} PATH=${VARIABLE_START}PATH${VARIABLE_END}${PATH_LIST_SEPARATOR}${CMAKE_INSTALL_PREFIX}${PATH_SEPARATOR}bin")

# Configure the environment variables file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/setup.in ${CMAKE_BINARY_DIR}/share/setup${SETUP_SCRIPT_EXTENSION})
INSTALL(FILES ${CMAKE_BINARY_DIR}/share/setup${SETUP_SCRIPT_EXTENSION} DESTINATION ${CMAKE_INSTALL_PREFIX}/share)
