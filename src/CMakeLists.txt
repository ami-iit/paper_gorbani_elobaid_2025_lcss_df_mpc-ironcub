
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

