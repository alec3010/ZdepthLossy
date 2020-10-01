cmake_minimum_required(VERSION 3.5)
project(zdepth LANGUAGES CXX)
include("cmake/zdepth-config-version.cmake")
set(CMAKE_CXX_STANDARD 11)


################################################################################
# Build Options

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

################################################################################
# Subprojects

if (NOT TARGET zstd)
    add_subdirectory(zstd zstd)
endif()
if (NOT TARGET codecs)
    add_subdirectory(codecs codecs)
endif()


################################################################################
# Source

set(INCLUDE_FILES
    include/zdepth.hpp
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/zdepth.cpp
)

include_directories(include)


################################################################################
# Targets

# zdepth library

add_library(zdepth STATIC ${SOURCE_FILES})
add_library(zdepth::zdepth ALIAS zdepth)

target_link_libraries(zdepth PUBLIC
    libzstd_shared
    codecs
)

target_include_directories(zdepth PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)


################################################################################
# Install

install(FILES LICENSE DESTINATION ".")
install(FILES README.md DESTINATION ".")
install(FILES ${INCLUDE_FILES} DESTINATION include)
