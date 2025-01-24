cmake_minimum_required(VERSION 3.5)
project(edge CXX)

include(FetchContent)

# find_package(OpenCV 4.6.0 REQUIRED)
# add_executable(edge  demo/main2.cpp)
# target_sources(edge PUBLIC demo/main2.cpp)

# FetchContent_Declare(trt-detect SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/trt-detect")
# FetchContent_MakeAvailable(trt-detect)

# target_include_directories(edge PUBLIC ${OPENCV_INCLUDE_DIRS})
# target_link_libraries(edge PUBLIC ${OpenCV_LIBS} trt-detect)

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)

find_package(OpenCV 4.6.0 REQUIRED)
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

# add_subdirectory(tkCommon)
add_subdirectory(tracker)

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 
                    ${CMAKE_CURRENT_SOURCE_DIR}/tkCommon/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/masa_protocol/include
                    /usr/include/gdal
                    ${EIGEN3_INCLUDE_DIR}
                    ${Python3_INCLUDE_DIRS}
                    ${OPENCV_INCLUDE_DIRS}
                    )

file(GLOB class-edge-SRC "src/*.cpp")

set(class-edge-LIBS ${OpenCV_LIBS} 
                    ${Python3_LIBRARIES}
                    gdal 
                    yaml-cpp 
                    ${ZeroMQ_LIBRARY}
                    )
add_library(class-edge ${class-edge-SRC})
# target_link_libraries(class-edge ${class-edge-LIBS} )

FetchContent_Declare(trt-detect SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/trt-detect/")
FetchContent_MakeAvailable(trt-detect)

target_include_directories(class-edge PUBLIC ${OPENCV_INCLUDE_DIRS})
target_link_libraries(class-edge PUBLIC ${class-edge-LIBS} trt-detect)

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(edge demo/main.cpp)
target_link_libraries(edge class-edge)
