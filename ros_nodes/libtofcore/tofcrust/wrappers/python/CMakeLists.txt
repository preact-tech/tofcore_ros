cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(ToFCrustPythonWrapper)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.10.4
)

FetchContent_MakeAvailable(pybind11)

pybind11_add_module(pytofcrust py_tofcrust.cpp)
set_target_properties(pytofcrust PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
target_link_libraries(pytofcrust PRIVATE tofcrust)

if(UNIX AND NOT APPLE)
    set_target_properties(pytofcrust PROPERTIES INSTALL_RPATH "$ORIGIN")
elseif(APPLE)
    set_target_properties(pytofcrust PROPERTIES INSTALL_RPATH "@loader_path")
endif()

set_target_properties(pytofcrust PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

target_compile_definitions(pytofcrust
                        PRIVATE VERSION_INFO=${VERSION_PACKAGE})
