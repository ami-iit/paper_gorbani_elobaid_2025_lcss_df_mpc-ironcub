
# Create the Python module using Pybind11
pybind11_add_module(bindingsMPC MPCPyBindings.cpp)

set(MPC_PYTHON_BINDING_DIR "${CMAKE_BINARY_DIR}/momentum_based_mpc")
set(MPC_BINDING_INSTALL_DIR "${PYTHON_INSTALL_DIR}/momentum_based_mpc")

# Link your C++ library to the Python module
target_link_libraries(bindingsMPC PUBLIC ${CPP_FLIGHT_LIBRARY_NAME} linearMomentumBasedMPC)
set_target_properties(bindingsMPC PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/MPCPyBindings OUTPUT_NAME "bindingsMPC")

install(TARGETS bindingsMPC DESTINATION ${MPC_BINDING_INSTALL_DIR} COMPONENT python)

file(WRITE ${MPC_PYTHON_BINDING_DIR}/__init__.py "")

# If we are on Windows and BUILD_SHARED_LIBS is ON, handle the fact that
# the Python interpreter does not look into PATH to find dll (see https://docs.python.org/3.8/library/os.html#os.add_dll_directory)
if(WIN32 AND BUILD_SHARED_LIBS)
    if(IS_ABSOLUTE MPC_BINDING_INSTALL_DIR)
        set(PYTHON_FULL_INSTDIR "${MPC_BINDING_INSTALL_DIR}")
    else()
        set(PYTHON_FULL_INSTDIR "${CMAKE_INSTALL_PREFIX}/${MPC_BINDING_INSTALL_DIR}")
    endif()
    file(RELATIVE_PATH RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY ${PYTHON_FULL_INSTDIR} ${CMAKE_INSTALL_FULL_BINDIR})
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "import os${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "library_dll_path = os.path.join(os.path.dirname(__file__),'${RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY}')${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "# Avoid to call add_dll_directory if not necessary,${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "# for example if the library to find are already found in the proper location in a conda environment${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "if(library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'Library','bin') and library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'bin')):${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "    if(os.path.exists(library_dll_path)):${NEW_LINE}")
    file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "        os.add_dll_directory(library_dll_path)${NEW_LINE}")
endif()

file(APPEND "${MPC_PYTHON_BINDING_DIR}/__init__.py" "from .bindingsMPC import *${NEW_LINE}")

# install the __init__.py file
install(FILES "${MPC_PYTHON_BINDING_DIR}/__init__.py" DESTINATION ${MPC_BINDING_INSTALL_DIR})
