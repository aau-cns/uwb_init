cmake_minimum_required(VERSION 3.5)
project(
    uwb_init_lib
    VERSION 1.0
    DESCRIPTION "Library for automatic, robust, and accurate Ultra-Wide-Band (UWB) anchors initialization"
    LANGUAGES CXX
)

option(OPTION_BUILD_EXAMPLES "Build examples."                                 ON)
option(OPTION_BUILD_TESTS "Build examples."                                    ON)
option(UWB_INIT_USE_SYSTEM_EIGEN "Use system wide Eigen3 or search locally..." OFF)


# Include cmake modules
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "BUILD TESTS = ${OPTION_BUILD_TESTS}")

## Include libraries

if(UWB_INIT_USE_SYSTEM_EIGEN)
  include(CMakeFindDependencyMacro)
  find_package(Eigen3 3.4 REQUIRED)
  find_dependency(Eigen3 3.4)
  list(APPEND libs Eigen)
else()
  include(FetchContent)

  # Eigen
  FetchContent_Declare(
      Eigen
      GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
      GIT_TAG         3.4.0
      GIT_SHALLOW     TRUE
      GIT_PROGRESS    TRUE
  )
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  set(EIGEN_BUILD_DOC OFF)
  set(EIGEN_BUILD_PKGCONFIG OFF)
  FetchContent_MakeAvailable(Eigen)
  list(APPEND libs Eigen3::Eigen)
endif()

message(" **************************************")
message(" --- Main Dependencies:")
message(" * Eigen: ${EIGEN3_INCLUDE_DIR}, ${Eigen_VERSION}  ${Eigen3_VERSION}")
message(" **************************************")

## Include and set up googletest
if(${OPTION_BUILD_TESTS})
    include(FetchContent)
    FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

## Try to compile with newer versions of C++
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_STANDARD 17)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no c++17 support. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Set compiler ${CMAKE_CXX_COMPILER}.")

## Define build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
# Set CMAKE compile flags
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif ()

message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}.")
message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}.")
message(STATUS "EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}.")


## List library source files
list(
    APPEND lib_sources
    source/utils/utils.cpp
    source/options/BiasType.cpp
    source/options/RANSAC_Options.cpp
    source/uwb_init.cpp
    source/solvers/linear/ls_solver.cpp
    source/solvers/nonlinear/nls_solver.cpp
    source/planners/wps_gen.cpp

)

## Include GNU dirs
include(GNUInstallDirs)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC ${lib_sources})

## include directories
## Specify generator expression to indicate build path and install path
## to allow easy re-location
target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${libs}

  INTERFACE
  $<BUILD_INTERFACE:${libs}>
)


# Examples
if(OPTION_BUILD_EXAMPLES)
    set(IDE_FOLDER "Examples")
    add_subdirectory(examples)
endif()


## Declare C++ tests
if(${OPTION_BUILD_TESTS})
    add_executable(test_wps test/test_wps_gen.cpp)
    #target_include_directories(test_wps PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_wps ${PROJECT_NAME}
)
    add_executable(test_utils test/test_utils.cpp)
    #target_include_directories(test_utils) # PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_utils ${PROJECT_NAME} GTest::gtest_main
)
    add_executable(test_init test/test_init.cpp)
    #target_include_directories(test_init PRIVATE) # ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(test_init ${PROJECT_NAME}
)

    include(GoogleTest)
    gtest_discover_tests(test_utils)
endif()

## Install
set(PROJECT_TARGET ${PROJECT_NAME}_targets)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_TARGET}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT ${PROJECT_TARGET}
        FILE ${PROJECT_TARGET}.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING
        PATTERN *.h
        PATTERN *.hpp
)

## Package
include(CMakePackageConfigHelpers)
set(PROJECT_CONFIG ${PROJECT_NAME}Config)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_CONFIG}.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
    "${PROJECT_CONFIG}Version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_CONFIG}.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_CONFIG}Version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
