cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(Scanner)

if (ENABLE_TRIGGER)
    find_package(Pylon QUIET)
    if (NOT ${Pylon_FOUND})
        include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
    endif()
    include_directories(${Pylon_INCLUDE_DIRS})

    FIND_PACKAGE(OpenCV REQUIRED)
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

    add_definitions(-DENABLE_TRIGGER)

    add_library(Capturer SHARED capturer.cpp ImageEventHandler.cpp sbaslercameracontrol.cpp)
    target_link_libraries(Capturer ${Pylon_LIBRARIES} ${OpenCV_LIBS})

else()
    add_library(Capturer SHARED capturer.cpp)
endif()

if (ENABLE_COMPUTATION)
    FIND_PACKAGE(OpenCV REQUIRED)
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

    FIND_PACKAGE(CUDA REQUIRED)
    INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --use_fast_math -gencode arch=compute_75,code=sm_75)
    add_definitions(-DENABLE_COMPUTATION)

    cuda_add_library(Matcher SHARED matcher.cpp PhaseCompute.cu)
    target_link_libraries(Matcher ${OpenCV_LIBS})

else()
    add_library(Matcher SHARED matcher.cpp)
endif()