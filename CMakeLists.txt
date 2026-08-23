cmake_minimum_required(VERSION 3.21)

set(PROJECT_NAME compg)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(compg)

set(PROJECT_PATH                               "${CMAKE_CURRENT_SOURCE_DIR}")
set(SRC_PATH                                   "${PROJECT_PATH}/src")
set(INCLUDE_PATH                               "${PROJECT_PATH}/include")
set(TEST_PATH                                  "${PROJECT_PATH}/test")
set(LIB_PATH                                   "${PROJECT_BINARY_DIR}/lib")
set(LIB_NAME                                   "${PROJECT_NAME}")
set(BIN_PATH                                   "${PROJECT_BINARY_DIR}/bin")
set(MAIN_NAME                                  "${PROJECT_NAME}_main")
set(TEST_NAME                                  "${PROJECT_NAME}_test")

OPTION(BUILD_MAIN                              "Build main"                     ON)
OPTION(BUILD_TEST                              "Build test"                     ON)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "RELEASE version selected.")
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)


set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Woverloaded-virtual -Wnon-virtual-dtor -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-changes-meaning")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
include(ProjectFiles.cmake)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIB_PATH})
add_library(${LIB_NAME} ${SRC})
target_include_directories(${LIB_NAME} PUBLIC ${INCLUDE_PATH} PUBLIC Eigen3::Eigen)
target_link_libraries(${LIB_NAME} stdc++exp)

if(BUILD_MAIN)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})
    add_executable(${MAIN_NAME} ${MAIN_SRC})
    target_link_libraries(${MAIN_NAME} ${LIB_NAME})

endif(BUILD_MAIN)


if(BUILD_TEST)
    Include(FetchContent)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        v3.8.1
    )
    FetchContent_MakeAvailable(Catch2)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} PRIVATE Catch2::Catch2WithMain ${LIB_NAME})
endif(BUILD_TEST)
