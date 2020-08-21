cmake_minimum_required(VERSION 3.5)
project(nvcuvid LANGUAGES CXX C)

################################################################################
# Dependencies

# CUDA
find_package(CUDA REQUIRED)

<<<<<<< HEAD
# NVIDIA Encoder Library
set(NVCUVID_LIB "${CMAKE_CURRENT_LIST_DIR}/vendored/linux/stubs/x86_64/libnvcuvid.so")

# CUDA Driver API
set(CUDA_LIB "${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs/libcuda.so")

# Print out directories 
message("NVCUVID_LIB: ${NVCUVID_LIB}")
message("CUDA_LIB: ${CUDA_LIBRARIES}")
message("CUDA_INCLUDE_DIR: ${CUDA_INCLUDE_DIRS}")



################################################################################
# Source

set(INCLUDE_FILES
    include/cuviddec.h
    include/nvcuvid.h
    include/NvDecoder.h
)
set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/NvDecoder.cpp
    src/NvEncoder.cpp
    src/NvEncoderCuda.cpp
)

include_directories(include ${CUDA_INCLUDE_DIRS})


################################################################################
# Targets

# nvcuvid library
add_library(nvcuvid STATIC ${SOURCE_FILES})
target_include_directories(nvcuvid PUBLIC include ${CUDA_INCLUDE_DIRS})

target_link_libraries(nvcuvid PUBLIC
    ${NVCUVID_LIB}
    ${CUDA_LIBRARIES}
    ${CUDA_LIB}
)

install(FILES ${INCLUDE_FILES} DESTINATION include)
install(TARGETS nvcuvid DESTINATION lib)
