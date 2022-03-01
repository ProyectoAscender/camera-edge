cmake_minimum_required(VERSION 3.5)
project(class-edge CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTKPROJ_PATH='\"${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/\"'")

#-------------------------------------------------------------------------------
# CUDA
#-------------------------------------------------------------------------------
find_package(CUDA 9.0 REQUIRED)
SET(CUDA_SEPARABLE_COMPILATION ON)
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_30 --compiler-options '-fPIC'")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS})

#file(GLOB class-edge-CUSRC "src/*.cu")
#cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDA_INCLUDE_DIRS} )
cuda_add_library(class-kernels src/undistort.cu)

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

find_package(yaml-cpp REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development)
message("Python_FOUND:${Python3_FOUND}")
message("Python_VERSION:${Python3_VERSION}")
message("Python_Development_FOUND:${Python3_Development_FOUND}")
message("Python_LIBRARIES:${Python3_LIBRARIES}")
message("Python_INCLUDE_DIRS:${Python3_INCLUDE_DIRS}")

## use pkg-config to get hints for 0mq locations
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)
## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )
## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

#-------------------------------------------------------------------------------
# Submodules
#-------------------------------------------------------------------------------

set (WITH_MATPLOTLIB OFF CACHE BOOL "Disable matplotlib")

add_subdirectory(tkDNN)
add_subdirectory(tkCommon)
add_subdirectory(class-tracker)

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
                    ${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/class-tracker/geodetic_utils/geodetic_utils/include/geodetic_utils 
                    ${CMAKE_CURRENT_SOURCE_DIR}/tkDNN/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol/include
                    /usr/include/gdal
                    ${EIGEN3_INCLUDE_DIR}
                    ${Python3_INCLUDE_DIRS}
                    ${OPENCV_INCLUDE_DIRS}
                    ${CUDA_INCLUDE_DIRS}  
                    ${NVINFER_INCLUDES} 
                    ${CUDNN_INCLUDE_DIRS}
                    )

file(GLOB class-edge-SRC "src/*.cpp")

set(class-edge-LIBS ${CUDA_LIBRARIES} 
                    ${CUDA_CUBLAS_LIBRARIES} 
                    ${CUDNN_LIBRARIES} 
                    ${OpenCV_LIBS} 
                    ${Python3_LIBRARIES}
                    gdal 
                    yaml-cpp 
                    tkDNN 
                    tkCommon 
                    class-tracker 
                    class-kernels 
                    ${ZeroMQ_LIBRARY}
                    )
add_library(class-edge SHARED ${class-edge-SRC})
target_link_libraries(class-edge ${class-edge-LIBS} )

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(edge demo/main.cpp)
target_link_libraries(edge class-edge)

add_executable(encrypt demo/encrypt_params.cpp)
target_link_libraries(encrypt class-edge)

add_executable(test_undistort demo/test_undistort.cpp)
target_link_libraries(test_undistort class-edge)
