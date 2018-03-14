# Specify the minimum required version of CMake to build the project.
cmake_minimum_required( VERSION 3.4 )
project( rscapture )

list( APPEND CMAKE_PREFIX_PATH "$ENV{HT}/cmake")
find_package( Houdini REQUIRED )

# Alglib finder with IRS_PREFIX_PATH var:
set( IRS_INCLUDE_PATH "$ENV{IRS_PREFIX_PATH}/include")
set( IRS_LIB_PATH     "$ENV{IRS_PREFIX_PATH}/build")

if (NOT EXISTS ${IRS_INCLUDE_PATH})
	message(FATAL_ERROR "Specify Intel RealSense location with $IRS_PREFIX_PATH. CMake will exit now.")
else()
	message(STATUS "IRS_INCLUDE_PATH: ${IRS_INCLUDE_PATH}")
endif()

# Collect files.
include_directories( ${IRS_INCLUDE_PATH} )
link_directories( ${IRS_LIB_PATH} )


# set( CMAKE_BUILD_TYPE Debug)

# Add a library with two source files.
set( library_name SOP_RealsenseCapture )

add_library( ${library_name} SHARED
    src/SOP_IntelRSCapture.cpp
    src/SOP_IntelRSCapture.hpp
)

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( ${library_name} Houdini )
target_link_libraries( ${library_name} realsense2)

# Configure several common target properties, such as its output directory.
houdini_configure_target( ${library_name} )

