cmake_minimum_required(VERSION 3.5)
project(nvcuvid LANGUAGES CXX C)
find_package(CUDA REQUIRED)

# On Windows this requires CUDA version 10.1.
# Other platforms are not tested.


################################################################################
# Dependencies

if (WIN32)
    set(CMAKE_FIND_LIBRARY_PREFIXES "")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
else()
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")

endif()

# FIXME: Only 64-bit version works
set(LIB_SELECT "x64")

if (WIN32)
    set(NVCUVID_LIB "${CMAKE_CURRENT_LIST_DIR}/vendored/${LIB_SELECT}/nvcuvid.lib")
else()
    set(NVCUVID_LIB "/home/qbiik/compression_compare/src/compression_test/ZdepthLossy/codecs/nvcuvid/vendored/linux/stubs/x86_64/libnvcuvid.so")
    #find_library(NVCUVID_LIB
    #    NAMES
    #        nvcuvid
    #   PATHS
    #        "/home/qbiik/compression_compare/src/compression_test/ZdepthLossy/codecs/nvcuvid/vendored/x64"
    #    NO_CMAKE_SYSTEM_PATH
    #)
endif()



#add_library(cuda_nvcuvid SHARED IMPORTED
#            IMPORTED_LOCATION ${NVCUVID_LIB}
#)


# FIXME: Only CUDA v10.1 works
#find_library(CUDART_STATIC_LIB
#    NAMES
#        cudart
#    PATHS
#        "/usr/local/cuda-11.0/lib64/"
#    NO_CMAKE_SYSTEM_PATH
#)
#find_package(CUDA-Toolkit REQUIRED)
#find_library(CUDA_LIB
#    NAMES
#        cuda
#    HINTS
 #       "/usr/local/cuda-11.0/lib64/"
 #   NO_CMAKE_SYSTEM_PATH
#)
#find_library(NVJPEG_LIB nvjpeg ${CMAKE_CUDA_TOOLKIT_IMPLICIT_LINK_DIRECTORIES})
#find_library(CUDART_STATIC_LIB cudart_static ${CUDAToolkit_LIBRARY_DIR}})

#NAMESINSTALL(EXPORT) given unknown export

#if (WIN32)
#    set(CMAKE_FIND_LIBRARY_PREFIXES "")
#    set(CMAKE_FIND_LIBRARY_SUFFIXES ".dll")
#else()
#    set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
#    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
#endif()

# nvJPEG needs to be vendored because the CUDA Toolkit does not install it
# in a predictable location.
#find_library(NVJPEG_DYNLIB
#    NAMES
#        nvcuvid
#    PATHS
#        "${CMAKE_CURRENT_LIST_DIR}/vendored/${LIB_SELECT}"
#    NO_CMAKE_SYSTEM_PATH
#)

message("NVCUVID_LIB: ${NVCUVID_LIB}")
message("CUDART_STATIC_LIB: ${CUDA_cudart_static_LIBRARY}")
message("CUDA_LIB: ${CUDA_LIBRARIES}")
message("CUDA_TOOLKIT: ${CUDA_TOOLKIT_ROOT_DIR}")
message("CUDA_INCLUDE_DIR: ${CUDA_INCLUDE_DIRS}")
set(CUDA_LIB "${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs/libcuda.so")



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
    #${CUDA_cudart_static_LIBRARY}
    ${CUDA_LIBRARIES}
    
    ${CUDA_LIB}
)

install(FILES ${INCLUDE_FILES} DESTINATION include)
install(TARGETS nvcuvid DESTINATION lib)
