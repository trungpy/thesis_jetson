cmake_minimum_required(VERSION 3.10)
project(thesis LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find package for json
find_package(nlohmann_json REQUIRED HINTS /usr/lib/cmake)
# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find dependencies
include(FindCUDAConfig)
include(FindOpenCVConfig)
include(FindTensorRTConfig)

# Include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Find catkin FIRST - this is crucial for catkin projects
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

## Find OpenCV - Force specific version and prioritize custom installation
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4" CACHE PATH "Path to OpenCV config")
find_package(OpenCV 4.1 REQUIRED COMPONENTS 
    core imgproc imgcodecs highgui videoio dnn
)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Detect platform
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_NAME)
message(STATUS "System architecture: ${ARCH_NAME}")

# Set CUDA paths
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda" CACHE PATH "Path to CUDA toolkit")
message(STATUS "CUDA Toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")

# Enable CUDA with manual standard setting
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++14 -Wno-deprecated-gpu-targets")

# Find CUDA libraries explicitly
find_library(CUDA_CUDART_LIBRARY cudart
    PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

# TensorRT detection - Manual approach for better control
if(ARCH_NAME STREQUAL "aarch64")
    set(TensorRT_INCLUDE_DIR "/usr/include/aarch64-linux-gnu")
    set(TensorRT_LIB_DIR "/usr/lib/aarch64-linux-gnu")
else()
    set(TensorRT_INCLUDE_DIR "/usr/include/x86_64-linux-gnu")
    set(TensorRT_LIB_DIR "/usr/lib/x86_64-linux-gnu")
endif()

# Allow user override
set(TensorRT_INCLUDE_DIR ${TensorRT_INCLUDE_DIR} CACHE PATH "TensorRT include directory")
set(TensorRT_LIB_DIR ${TensorRT_LIB_DIR} CACHE PATH "TensorRT library directory")

# Check TensorRT header
if(NOT EXISTS "${TensorRT_INCLUDE_DIR}/NvInfer.h")
    message(FATAL_ERROR "TensorRT header (NvInfer.h) not found in ${TensorRT_INCLUDE_DIR}")
endif()

# Find TensorRT libraries explicitly
set(TensorRT_LIBRARIES)
set(TensorRT_LIBS nvinfer nvinfer_plugin nvonnxparser)
foreach(lib ${TensorRT_LIBS})
    find_library(TensorRT_${lib}_LIBRARY 
        NAMES ${lib}
        PATHS ${TensorRT_LIB_DIR}
        NO_DEFAULT_PATH
    )
    if(TensorRT_${lib}_LIBRARY)
        list(APPEND TensorRT_LIBRARIES ${TensorRT_${lib}_LIBRARY})
        message(STATUS "Found TensorRT ${lib}: ${TensorRT_${lib}_LIBRARY}")
    else()
        message(FATAL_ERROR "TensorRT library ${lib} not found in ${TensorRT_LIB_DIR}")
    endif()
endforeach()

# Set CUDA architectures
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    if(ARCH_NAME STREQUAL "aarch64")
        set(CUDA_ARCH_LIST "53;72") # Jetson Nano (53) and Xavier/Orin (72)
    else()
        set(CUDA_ARCH_LIST "75;80;86") # Modern NVIDIA architectures
    endif()
endif()

# Gather all cpp/cu files
file(GLOB_RECURSE ALL_SOURCES src/*.cpp src/modules/*.cpp src/*.cu)

# Define entry points that should be excluded from shared sources
set(VIDEO_PUBLISHER_ENTRY src/video_publisher.cpp)
set(VIDEO_SUBSCRIBER_ENTRY src/video_subscriber.cpp)
set(VIDEO_SUBSCRIBER_CAPTURE_ENTRY src/video_subscriber_capture.cpp)


set(ECHO_EGO_SPEED_ENTRY src/echo_ego_speed.cpp)
set(ECHO_DRIVING_ACTION_ENTRY src/echo_driving_action.cpp)
set(ECHO_THROTTLE_ENTRY src/echo_throttle.cpp)
set(ECHO_BRAKE_ENTRY src/echo_brake.cpp)

# Remove ALL entry points from shared sources
set(SHARED_SOURCES ${ALL_SOURCES})
list(REMOVE_ITEM SHARED_SOURCES 
     ${CMAKE_CURRENT_SOURCE_DIR}/${VIDEO_PUBLISHER_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${VIDEO_SUBSCRIBER_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${VIDEO_SUBSCRIBER_CAPTURE_ENTRY}
     ${VIDEO_PUBLISHER_ENTRY}
     ${VIDEO_SUBSCRIBER_ENTRY}
     ${VIDEO_SUBSCRIBER_CAPTURE_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${ECHO_EGO_SPEED_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${ECHO_DRIVING_ACTION_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${ECHO_THROTTLE_ENTRY}
     ${CMAKE_CURRENT_SOURCE_DIR}/${ECHO_BRAKE_ENTRY}
     ${ECHO_EGO_SPEED_ENTRY}
     ${ECHO_DRIVING_ACTION_ENTRY}
     ${ECHO_THROTTLE_ENTRY}
     ${ECHO_BRAKE_ENTRY}
)

# Debug: Print what's in SHARED_SOURCES to verify
message(STATUS "SHARED_SOURCES contains: ${SHARED_SOURCES}")

# Only add Detect.cpp if it exists and not already in SHARED_SOURCES
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/Detect.cpp")
    list(FIND SHARED_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/Detect.cpp" DETECT_FOUND)
    if(DETECT_FOUND EQUAL -1)
        list(APPEND SHARED_SOURCES src/Detect.cpp)
    endif()
else()
    message(WARNING "src/Detect.cpp not found. Please check your sources.")
endif()

###################################
## catkin specific configuration ##
###################################

# Catkin package setup - MUST come before any add_executable or add_library
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bytetrack
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  DEPENDS OpenCV TensorRT
)

###########
## Build ##
###########

# Include directories - prioritize OpenCV 4.1
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Bytetrack library
include_directories(${PROJECT_SOURCE_DIR}/bytetrack/include)
file(GLOB_RECURSE SRCS01 ${PROJECT_SOURCE_DIR}/bytetrack/src/*.cpp)
add_library(bytetrack SHARED ${SRCS01})
target_link_libraries(bytetrack 
    ${CUDA_CUDART_LIBRARY} 
    ${TensorRT_LIBRARIES}
    ${OpenCV_LIBS}
)

# Set CUDA properties for bytetrack if it has CUDA sources
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    set_target_properties(bytetrack PROPERTIES
        CUDA_ARCHITECTURES "${CUDA_ARCH_LIST}"
    )
endif()

# Publisher executable (ROS specific)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/video_publisher.cpp")
    add_executable(video_publisher src/video_publisher.cpp)
    target_link_libraries(video_publisher 
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
    )
endif()

# Publisher executable (ROS specific)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/video_subscriber_capture.cpp")
    add_executable(video_subscriber_capture src/video_subscriber_capture.cpp)
    target_link_libraries(video_subscriber_capture 
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBS}
    )
endif()


# Subscriber executable with CUDA (ROS specific)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/video_subscriber.cpp")
    add_executable(video_subscriber src/video_subscriber.cpp ${SHARED_SOURCES})
    target_link_libraries(video_subscriber 
        PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${TensorRT_LIBRARIES}
        ${CUDA_CUDART_LIBRARY}
        bytetrack 
        nlohmann_json
    )

    # Set CUDA properties for video_subscriber
    set_target_properties(video_subscriber PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
        set_target_properties(video_subscriber PROPERTIES
            CUDA_ARCHITECTURES "${CUDA_ARCH_LIST}"
        )
    endif()

    # Set RPATH for video_subscriber
    set_target_properties(video_subscriber PROPERTIES
        BUILD_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
        INSTALL_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
    )
endif()

# Set RPATH for ROS executables
if(TARGET video_subscriber)
    set_target_properties(video_subscriber PROPERTIES
        BUILD_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
        INSTALL_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
    )
endif()


add_executable(echo_ego_speed ${ECHO_EGO_SPEED_ENTRY})
add_executable(echo_driving_action ${ECHO_DRIVING_ACTION_ENTRY})
add_executable(echo_throttle ${ECHO_THROTTLE_ENTRY})
add_executable(echo_brake ${ECHO_BRAKE_ENTRY})

target_link_libraries(echo_ego_speed ${catkin_LIBRARIES})
target_link_libraries(echo_driving_action ${catkin_LIBRARIES})
target_link_libraries(echo_throttle ${catkin_LIBRARIES})
target_link_libraries(echo_brake ${catkin_LIBRARIES})
