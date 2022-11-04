cmake_minimum_required(VERSION 3.14)
project(
    uwb_init_lib
    VERSION 1.0
    DESCRIPTION "Library for automatic, robust, and accurate Ultra-Wide-Band (UWB) anchors initialization"
    LANGUAGES CXX
)

# Include libraries
find_package(Boost REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

# Include and set up googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

## Display packages version
message(STATUS "EIGEN VERSION: " ${Eigen3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Try to compile with newer versions of C++
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    set(CMAKE_CXX_STANDARD 20)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no c++20 support. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Set compiler ${CMAKE_CXX_COMPILER}.")

# Define build type
#set(CMAKE_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
# Set CMAKE compile flags
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif ()
message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}.")

# Include header files
include_directories(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    include
)

# List third party libraries used by all binaries
list(
    APPEND thirdparty_libraries
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# List library source files
list(
    APPEND lib_sources
    source/uwb_init.cpp
)

## Declare a C++ library
add_library(uwb_init SHARED ${lib_sources})
target_link_libraries(uwb_init ${thirdparty_libraries})

## Declare C++ tests
add_executable(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${thirdparty_libraries} uwb_init GTest::gtest_main)
add_executable(test_init test/test_init.cpp)
target_link_libraries(test_init ${thirdparty_libraries} uwb_init)
add_executable(test_eigen test/test_eigen.cpp)
target_link_libraries(test_eigen ${thirdparty_libraries} uwb_init)

include(GoogleTest)
gtest_discover_tests(test_utils)

