###############################################################################
# CMakeList.txt file for the libtofcrust library
# 
#  See the links below for the how and why of this CMake file structure
#  - https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/ and 
#  - https://github.com/pabloariasal/modern-cmake-sample/tree/master/libjsonutils
##
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(libToFCrust VERSION 1.2.0 LANGUAGES CXX)

#Make sure that custom modules are found
list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

include(cmake/tof_options.cmake)

###########################
# Find system dependencies
set(MIN_BOOST_VERSION 1.70)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED COMPONENTS)
###########################

add_library(tofcrust)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(ToFCrust::tofcrust ALIAS tofcrust)

###################
# Compiler Options:
###################
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(tofcrust PUBLIC -Wall -Wextra)
endif()
if(MSVC)
    target_compile_definitions(tofcore PUBLIC WINVER=0x0A00 _WIN32_WINNT=0x0A00)
endif()

target_compile_features(tofcrust PUBLIC cxx_std_17)


target_sources(tofcrust
  PRIVATE
    src/tof_sensor.cpp
)

#Set target properties
target_include_directories(tofcrust
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tofcrust>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(tofcrust
  PUBLIC
  ToFCore::tofcore
  PRIVATE
  # Boost::regex example of private dependency
)

include(GenerateExportHeader)
generate_export_header(tofcrust)

##############################################
# Installation instructions

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/tofcrust)

if (WIN32)
    install(TARGETS tofcrust
        EXPORT tofcrust-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
else()
    install(TARGETS tofcrust
        EXPORT tofcrust-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

#This is required so that the exported target has the name ToFCrust and not tofcrust
set_target_properties(tofcrust PROPERTIES EXPORT_NAME ToFCrust)

# fixes linking issue with python bindings
set_target_properties(tofcrust PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

if (WIN32)
    set_target_properties(tofcrust PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/tofcrust_export.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/tofcrust")

#Export the targets to a script
install(EXPORT tofcrust-targets
    FILE
        ToFCrustTargets.cmake
    NAMESPACE
        ToFCrust::
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

#Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCrustConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/ToFCrustConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCrustConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

#Install the config, configversion and custom find modules
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCrustConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCrustConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
## Exporting from the build tree
export(EXPORT tofcrust-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/ToFCrustTargets.cmake
    NAMESPACE ToFCrust::)

#Register package in the User Package Registry
export(PACKAGE ToFCrust)

##############################################
## Add test
enable_testing()
add_subdirectory(wrappers)
add_subdirectory(test)
