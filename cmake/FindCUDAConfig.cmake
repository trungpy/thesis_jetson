# cmake/FindCUDAConfig.cmake
find_package(CUDA REQUIRED)

# Set CUDA include path explicitly
if(NOT CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
endif()

message(STATUS "CUDA toolkit path: ${CUDA_TOOLKIT_ROOT_DIR}")

# Add CUDA include directories
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)

# Set CUDA architectures
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_NAME)
if(ARCH_NAME STREQUAL "aarch64")
    set(CUDA_ARCH_LIST "53" CACHE STRING "CUDA architectures to build for")
else()
    set(CUDA_ARCH_LIST "75;80;86" CACHE STRING "CUDA architectures to build for")
endif()

# Common CUDA flags
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Wno-deprecated-gpu-targets")

if(ARCH_NAME STREQUAL "aarch64")
    add_compile_options(-Wno-deprecated-declarations)
endif()