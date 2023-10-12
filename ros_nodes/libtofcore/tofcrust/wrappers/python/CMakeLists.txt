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
target_link_libraries(pytofcrust PRIVATE tofcrust)

target_compile_definitions(pytofcrust
                        PRIVATE VERSION_INFO=${VERSION_PACKAGE})
