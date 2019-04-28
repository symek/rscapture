 # Specify the minimum required version of CMake to build the project.
cmake_minimum_required( VERSION 3.4 )
project( rscapture )

list( APPEND CMAKE_PREFIX_PATH "$ENV{HT}/cmake")
find_package( Houdini REQUIRED )

# Intel Realsense finder with IRS_PREFIX_PATH var:
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
set( realsense_dso SOP_RealsenseCapture )

add_library( ${realsense_dso} SHARED
    src/SOP_IntelRSCapture.cpp
    src/SOP_IntelRSCapture.hpp
)

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( ${realsense_dso} Houdini )
target_link_libraries( ${realsense_dso} realsense2)

# Configure several common target properties, such as its output directory.
houdini_configure_target( ${realsense_dso} )


# Intel Realsense finder with IRS_PREFIX_PATH var:
#set( SPARSEICP_INCLUDE_PATH "$ENV{SPARSEICP_PREFIX_PATH}")
#set( NANOFLANN_INCLUDE_PATH "$ENV{SPARSEICP_PREFIX_PATH}/include")
#
#if (NOT EXISTS ${SPARSEICP_INCLUDE_PATH})
#	message(FATAL_ERROR "Specify SPARSEICP location with $SPARSEICP_PREFIX_PATH. CMake will exit now.")
#else()
#	message(STATUS "SPARSEICP_INCLUDE_PATH: ${SPARSEICP_INCLUDE_PATH}")
#	message(STATUS "NANOFLANN_INCLUDE_PATH: ${NANOFLANN_INCLUDE_PATH}")
#endif()
#
#include_directories( ${SPARSEICP_INCLUDE_PATH} )
#include_directories( ${NANOFLANN_INCLUDE_PATH} )
#
## Eigen time
#set( EIGEN_INCLUDE_DIR "$ENV{EIGEN_INCLUDE_DIR}")
#if (NOT EXISTS ${EIGEN_INCLUDE_DIR})
#	message(FATAL_ERROR "Specify EIGEN location with $EIGEN_INCLUDE_DIR. CMake will exit now.")
#else()
#	message(STATUS "EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}")
#endif()
#include_directories(${EIGEN_INCLUDE_DIR})
#
#
## PCL time
#find_package(PCL 1.3 REQUIRED COMPONENTS common features)
#
#set( EIGEN_INCLUDE_DIR "$ENV{EIGEN_INCLUDE_DIR}")
#if (NOT EXISTS ${EIGEN_INCLUDE_DIR})
#	message(FATAL_ERROR "Specify EIGEN location with $EIGEN_INCLUDE_DIR. CMake will exit now.")
#else()
#	message(STATUS "EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}")
#endif()
#
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#
#set( pcallign_dso SOP_PCAllign)
#
#add_library( ${pcallign_dso} SHARED
#	src/intel_fgr.hpp
#	src/intel_fgr.cpp
#	src/pcl_fpfh.hpp
#	src/SOP_PCAlign.cpp
#	src/SOP_PCAlign.hpp)
#
#target_link_libraries( ${pcallign_dso} Houdini)
#target_link_libraries( ${pcallign_dso} ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES})
#houdini_configure_target( ${pcallign_dso} )



