cmake_minimum_required(VERSION 3.3)
project(semi_global_matching)


find_package(OpenCV 4 REQUIRED)


option(ENABLE_CMAKE_DEBUG "Enable detailed debug messages" ON)

if(ENABLE_CMAKE_DEBUG)
    # Debug or Release
    set(CMAKE_BUILD_TYPE "Debug")
else()
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS_DEBUG "-g3")



list(APPEND LIBRARY_THIRDPARTY
    ${OpenCV_LIBS}
)


list(APPEND DIRS_HEADER_THIRDPARTY
    ${OpenCV_INCLUDE_DIRS}
)


set(LIBRARY_SOURCES
    SemiGlobalMatching.cpp
    sgm_util.cpp
    stdafx.cpp
)

add_executable(
    run_semi_global_matching
    main.cpp
    ${LIBRARY_SOURCES}
)

target_include_directories(run_semi_global_matching
    PUBLIC
    ${DIRS_HEADER_THIRDPARTY}
)

target_link_libraries(run_semi_global_matching
    PUBLIC
    ${LIBRARY_THIRDPARTY}
)
