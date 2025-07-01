cmake_minimum_required(VERSION 3.10)
project(thesis LANGUAGES CXX CUDA)

## Compile as C++14 (reliable with CUDA 11.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find dependencies
include(FindCUDAConfig)
include(FindOpenCVConfig)
include(FindTensorRTConfig)



## Find catkin
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
# Don't set CMAKE_CUDA_STANDARD, use manual flags instead
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++14 -Wno-deprecated-gpu-targets")

# Suppress deprecated GPU architecture warnings - already included above

# Find CUDA libraries explicitly
find_library(CUDA_CUDART_LIBRARY cudart
    PATHS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

# Source files
set(HEADERS
    src/Detect.h
    src/preprocess.h
)

set(SOURCES
    src/video_subscriber.cpp
    src/preprocess.cu
)

# Only add Detect.cpp if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/Detect.cpp")
    list(APPEND SOURCES src/Detect.cpp)
else()
    message(WARNING "src/Detect.cpp not found. Please check your sources.")
endif()

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

# Static library with CUDA support
set(LIB_SOURCE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/lanevision"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/followdist"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt"
)

# Collect library sources
set(LIB_SOURCES)
foreach(dir ${LIB_SOURCE_DIRS})
    if(EXISTS ${dir})
        file(GLOB dir_sources 
            "${dir}/*.cpp"
            "${dir}/*.cu"
            "${dir}/*.cxx"
        )
        list(APPEND LIB_SOURCES ${dir_sources})
    endif()
endforeach()

message(STATUS "LIB_SOURCES: ${LIB_SOURCES}")

# Create library only if we have sources
if(LIB_SOURCES)
    add_library(acc STATIC ${LIB_SOURCES})
    target_include_directories(acc PUBLIC
        ${LIB_SOURCE_DIRS}
        ${TensorRT_INCLUDE_DIR}
        ${CUDA_TOOLKIT_ROOT_DIR}/include
    )
    set(HAVE_ACC_LIB TRUE)
else()
    message(WARNING "No library sources found, creating empty library")
    add_library(acc STATIC src/empty.cpp)  # You'll need to create this empty file
    set(HAVE_ACC_LIB FALSE)
endif()

# Catkin package setup
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES acc
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
)

###########
## Build ##
###########

# Include directories - prioritize OpenCV 4.1
include_directories(
  ${OpenCV_INCLUDE_DIRS}  # Put OpenCV first
  include
  ${catkin_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIR}
  ${CUDA_TOOLKIT_ROOT_DIR}/include
)

# Publisher executable
add_executable(video_publisher src/video_publisher.cpp)
target_link_libraries(video_publisher 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)

# Subscriber executable with CUDA
add_executable(video_subscriber ${SOURCES})

# Link libraries - prioritize OpenCV 4.1 and be explicit about ordering
target_link_libraries(video_subscriber 
  PRIVATE
  acc 
  ${OpenCV_LIBS}  # Link OpenCV first to prioritize 4.1
  ${catkin_LIBRARIES} 
  ${TensorRT_LIBRARIES}
  ${CUDA_CUDART_LIBRARY}
)

# Set CUDA properties
set_target_properties(video_subscriber PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# Set CUDA architectures
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    if(ARCH_NAME STREQUAL "aarch64")
        set(CUDA_ARCH_LIST "53;72") # Jetson Nano (53) and Xavier/Orin (72)
    else()
        set(CUDA_ARCH_LIST "75;80;86") # Modern NVIDIA architectures
    endif()
    
    set_target_properties(video_subscriber PROPERTIES
        CUDA_ARCHITECTURES "${CUDA_ARCH_LIST}"
    )
endif()

# Set RPATH
set_target_properties(video_subscriber PROPERTIES
    BUILD_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
    INSTALL_RPATH "${TensorRT_LIB_DIR}:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
)