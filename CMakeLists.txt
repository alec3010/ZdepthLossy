cmake_minimum_required(VERSION 3.5)
project(zdepth LANGUAGES CXX)
include("cmake/zdepth-config-version.cmake")
include(GenerateExportHeader)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS true)


################################################################################
# Build Options

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# In debug mode, add -DDEBUG
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    # Warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

    # Remove Asio warning
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")

    # Static library: -fPIC
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -march=native -fstack-protector")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
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
    include/zdepthcaller.hpp
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/zdepth.cpp
    src/zdepthcaller.cpp

)

include_directories(include)


################################################################################
# Targets

# zdepth library

add_library(zdepth SHARED ${SOURCE_FILES})
generate_export_header(zdepth 
                BASE_NAME zdepth
                EXPORT_MACRO_NAME zdepth_EXPORT
                EXPORT_FILE_NAME zdepth_Export.h 
                STATIC_DEFINE zdepth_BUILT_AS_STATIC)
add_library(zdepth::zdepth ALIAS zdepth)

target_link_libraries(zdepth PUBLIC
    zstd
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

return()


# Disable the rest of this because it does not seem to work on Linux
install(
    TARGETS
        zdepth
        zstd
    EXPORT
        zdepth-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES cmake/zdepth-config.cmake DESTINATION cmake)
install(FILES cmake/zdepth-config-version.cmake DESTINATION cmake)
install(EXPORT zdepth-targets DESTINATION cmake)
install(FILES "$<TARGET_FILE:zdepth>" DESTINATION lib)
