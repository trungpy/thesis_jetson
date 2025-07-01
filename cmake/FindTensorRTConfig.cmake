# cmake/FindTensorRT.cmake
find_package(CUDA REQUIRED)

# Detect platform
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_NAME)

if(ARCH_NAME STREQUAL "aarch64")
    message(STATUS "Jetson detected - using TensorRT")
    set(TensorRT_INCLUDE_DIR "/usr/include/aarch64-linux-gnu")
    set(TensorRT_LIB_DIR "/usr/lib/aarch64-linux-gnu")
else()
    message(STATUS "x86_64 detected with TensorRT")
    set(TensorRT_INCLUDE_DIR "/usr/include/x86_64-linux-gnu")
    set(TensorRT_LIB_DIR "/usr/lib/x86_64-linux-gnu")
endif()

set(TensorRT_INCLUDE_DIR ${TensorRT_INCLUDE_DIR} CACHE PATH "TensorRT include directory")
set(TensorRT_LIB_DIR ${TensorRT_LIB_DIR} CACHE PATH "TensorRT library directory")

if(NOT EXISTS "${TensorRT_INCLUDE_DIR}/NvInfer.h")
    message(FATAL_ERROR "TensorRT header (NvInfer.h) not found in ${TensorRT_INCLUDE_DIR}")
endif()

set(TensorRT_LIBS nvinfer nvinfer_plugin nvonnxparser)
foreach(lib ${TensorRT_LIBS})
    find_library(TensorRT_${lib}_LIBRARY 
        NAMES ${lib} 
        PATHS ${TensorRT_LIB_DIR} 
        NO_DEFAULT_PATH
    )
    if(NOT TensorRT_${lib}_LIBRARY)
        message(FATAL_ERROR "TensorRT library ${lib} not found in ${TensorRT_LIB_DIR}")
    endif()
    list(APPEND TensorRT_LIBRARIES ${TensorRT_${lib}_LIBRARY})
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
    REQUIRED_VARS 
        TensorRT_INCLUDE_DIR 
        TensorRT_LIB_DIR 
        TensorRT_LIBRARIES
)

mark_as_advanced(
    TensorRT_INCLUDE_DIR
    TensorRT_LIB_DIR
    TensorRT_LIBRARIES
)