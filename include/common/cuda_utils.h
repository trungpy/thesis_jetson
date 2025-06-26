#ifndef TRTX_CUDA_UTILS_H_
#define TRTX_CUDA_UTILS_H_

#include <cuda_runtime_api.h>
#include <iostream>
#include <cassert>

// CUDA error checking macro
#ifndef CUDA_CHECK
#define CUDA_CHECK(call)                                                              \
    do {                                                                              \
        cudaError_t err = call;                                                      \
        if (err != cudaSuccess) {                                                    \
            std::cerr << "[CUDA ERROR] " << cudaGetErrorString(err)                  \
                      << " in file " << __FILE__                                     \
                      << " at line " << __LINE__ << std::endl;                       \
            assert(false);                                                           \
        }                                                                             \
    } while (0)
#endif  // CUDA_CHECK

#endif  // TRTX_CUDA_UTILS_H_
