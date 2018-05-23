project (cmt)

cmake_minimum_required (VERSION 3.1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

# set(OpenCV_DIR /opt/share/OpenCV/)
find_package(OpenCV REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_definitions ("-Wall")

set(CMAKE_CXX_STANDARD 11)

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable (${PROJECT_NAME} common.cpp gui.cpp main.cpp
    CMT.cpp Consensus.cpp Fusion.cpp Matcher.cpp Tracker.cpp
    fastcluster/fastcluster.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
