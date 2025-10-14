cmake_minimum_required(VERSION 3.4...3.18)
project(ORB_SLAM_PYTHON_BINDING)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ============================================================================
# BUILD OPTIONS
# ============================================================================
option(BUILD_ENHANCED_BINDINGS "Build enhanced Python bindings with hardware detection" ON)
option(ENABLE_CUDA "Enable CUDA support for hardware detection" OFF)

if(BUILD_ENHANCED_BINDINGS)
  message(STATUS "Building ENHANCED Python bindings with hardware adaptation")
  add_definitions(-DENHANCED_BINDINGS)
else()
  message(STATUS "Building LEGACY Python bindings (basic features only)")
endif()

if(ENABLE_CUDA)
  # Allow either -DCUDA_DIR=... or ENV{CUDA_DIR}
  if(DEFINED CUDA_DIR)
    set(_cuda_root "${CUDA_DIR}")
  elseif(DEFINED ENV{CUDA_DIR})
    set(_cuda_root "$ENV{CUDA_DIR}")
  endif()

  if(_cuda_root)
    message(STATUS "Setting CUDAToolkit root from CUDA_DIR: ${_cuda_root}")
    # Expose to CMake find_package as CUDAToolkit root
    set(CUDAToolkit_ROOT "${_cuda_root}" CACHE PATH "CUDA root (from CUDA_DIR or ENV{CUDA_DIR})" FORCE)
  endif()

  # Prefer modern CMake discovery (CMake >= 3.17). Use QUIET so we can fallback.
  find_package(CUDAToolkit QUIET)

  if(TARGET CUDAToolkit::cudart)
    message(STATUS "CUDA support ENABLED via CUDAToolkit (${CUDAToolkit_VERSION})")
    add_compile_definitions(-DENABLE_CUDA)
    set(CUDA_SUPPORT_ENABLED ON)
    # Note: linking will be done later via the imported target
  else()
    # Fallback to legacy FindCUDA if modern CUDAToolkit isn't available
    find_package(CUDA QUIET)
    if(CUDA_FOUND)
      message(STATUS "Found legacy CUDA via FindCUDA: ${CUDA_VERSION}")
      add_compile_definitions(-DENABLE_CUDA)
      set(CUDA_SUPPORT_ENABLED ON)
      # preserve legacy cudart var for linking
      set(CUDA_LINK_LIBS ${CUDA_CUDART_LIBRARY})
    else()
      message(WARNING "CUDA requested but not found; disabling CUDA support")
      set(CUDA_SUPPORT_ENABLED OFF)
    endif()
  endif()
else()
  set(CUDA_SUPPORT_ENABLED OFF)
endif()
# ============================================================================
# FIND DEPENDENCIES
# ============================================================================
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Pangolin (for visualization)
if(DEFINED ENV{Pangolin_DIR})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{Pangolin_DIR}")
  message(STATUS "Using Pangolin_DIR from environment: $ENV{Pangolin_DIR}")
endif()
find_package(Pangolin REQUIRED)
message(STATUS "Found Pangolin ${PANGOLIN_INCLUDE_DIR} ${PANGOLIN_LIBRARY}")

# Python & NumPy
find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Found Python ${Python_VERSION}")
find_package(NumPy REQUIRED)
if(NOT NumPy_INCLUDE_DIR AND DEFINED NUMPY_INCLUDE_DIR)
  set(NumPy_INCLUDE_DIR ${NUMPY_INCLUDE_DIR})
  message(STATUS "Using NumPy include directory from setup.py: ${NumPy_INCLUDE_DIR}")
endif()

# Boost.Python
find_package(Boost REQUIRED COMPONENTS python thread system serialization)
message(STATUS "Found Boost.Python component: ${Boost_LIBRARIES}")

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV ${OpenCV_VERSION}: ${OpenCV_LIBS}")

# Eigen3
find_package(Eigen3 3.3 QUIET)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3 ${Eigen3_VERSION}")
endif()


# ============================================================================
# GLOBAL COMPILER FLAGS
# ============================================================================
set(BASE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-maybe-uninitialized -Wno-deprecated-declarations")

# Architecture-specific optimizations
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
  set(ARCH_FLAGS "-O3 -march=armv8-a")
  message(STATUS "Target architecture: ARMv8 (aarch64)")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set(ARCH_FLAGS "-O3 -mfpu=neon -march=armv7-a")
  message(STATUS "Target architecture: ARMv7")
else() # x86_64
  set(ARCH_FLAGS "-O3 -mmmx -msse -msse2 -msse3 -mssse3")
  message(STATUS "Target architecture: x86_64")
endif()

# Append flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${BASE_CXX_FLAGS} ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BASE_CXX_FLAGS} ${ARCH_FLAGS}")


# ============================================================================
# ADD CORE ORB_SLAM3 LIBRARY
# ============================================================================
add_subdirectory(orbslam3)


# ============================================================================
# CREATE PYTHON MODULE
# ============================================================================
if(BUILD_ENHANCED_BINDINGS)
  set(PYTHON_MODULE_SOURCES
    src/ORBSlamPythonEnhanced.cpp
    src/pyboostcvconverter/pyboost_cv3_converter.cpp
  )
  set(PYTHON_MODULE_OUTPUT_NAME "orbslam3_enhanced")
  message(STATUS "Building enhanced module: ${PYTHON_MODULE_OUTPUT_NAME}.so")
else()
  set(PYTHON_MODULE_SOURCES
    src/ORBSlamPython.cpp
    src/pyboostcvconverter/pyboost_cv3_converter.cpp
  )
  set(PYTHON_MODULE_OUTPUT_NAME "orbslam3")
  message(STATUS "Building legacy module: ${PYTHON_MODULE_OUTPUT_NAME}.so")
endif()

# Create the Python module
set(PYTHON_MODULE_NAME orbslam3_python)
add_library(${PYTHON_MODULE_NAME} SHARED ${PYTHON_MODULE_SOURCES})

# Set module properties
set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES
  PREFIX ""                                    # No "lib" prefix
  OUTPUT_NAME ${PYTHON_MODULE_OUTPUT_NAME}     # Output name based on build type
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)


# ============================================================================
# CONFIGURE PYTHON MODULE
# ============================================================================
target_include_directories(${PYTHON_MODULE_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/orbslam3/include
  ${Python_INCLUDE_DIRS}
  ${NumPy_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)
if(Pangolin_FOUND)
  target_include_directories(${PYTHON_MODULE_NAME} PRIVATE ${Pangolin_INCLUDE_DIRS})
endif()
if(Eigen3_FOUND)
  target_include_directories(${PYTHON_MODULE_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# Link the python module against the core ORB_SLAM3 library and other dependencies
target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE
  ORB_SLAM3
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  # ${Python_LIBRARIES}
)
if(Pangolin_FOUND)
  target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE ${Pangolin_LIBRARIES})
endif()
if(CUDA_SUPPORT_ENABLED AND TARGET CUDAToolkit::cudart)
  target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE CUDAToolkit::cudart)
elseif(CUDA_SUPPORT_ENABLED AND DEFINED CUDA_LINK_LIBS)
  target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE ${CUDA_LINK_LIBS})
endif()

# Platform-specific libraries for hardware detection
if(UNIX AND NOT APPLE)
  # Linux - no additional libraries needed (uses /proc, sysinfo)
  target_compile_definitions(${PYTHON_MODULE_NAME} PRIVATE __linux__)
elseif(WIN32)
  # Windows - link psapi for memory info
  target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE psapi)
  target_compile_definitions(${PYTHON_MODULE_NAME} PRIVATE _WIN32)
elseif(APPLE)
  # macOS - link system framework
  target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE "-framework System")
  target_compile_definitions(${PYTHON_MODULE_NAME} PRIVATE __APPLE__)
endif()


# ============================================================================
# OUTPUT PATHS
# ============================================================================
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# ============================================================================
# INSTALLATION (Optional)
# ============================================================================

# Uncomment if you want to install the module system-wide
# install(TARGETS ${PYTHON_MODULE_NAME}
#   LIBRARY DESTINATION ${Python_SITEARCH}/orbslam3
# )


# ============================================================================
# BUILD SUMMARY
# ============================================================================

message(STATUS "")
message(STATUS "=================================================================")
message(STATUS "ORB-SLAM3 Python Bindings Configuration Summary")
message(STATUS "=================================================================")
message(STATUS "Build Type:              ${CMAKE_BUILD_TYPE}")
message(STATUS "Enhanced Bindings:       ${BUILD_ENHANCED_BINDINGS}")
message(STATUS "CUDA Support:            ${ENABLE_CUDA}")
message(STATUS "Module Name:             ${PYTHON_MODULE_OUTPUT_NAME}.so")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  - Python:              ${Python_VERSION}")
message(STATUS "  - NumPy:               ${NumPy_INCLUDE_DIR}")
message(STATUS "  - Boost:               ${Boost_VERSION}")
message(STATUS "  - OpenCV:              ${OpenCV_VERSION}")
message(STATUS "  - Pangolin:            ${Pangolin_VERSION}")
if(Eigen3_FOUND)
message(STATUS "  - Eigen3:              ${Eigen3_VERSION}")
endif()
if(CUDA_SUPPORT_ENABLED)
message(STATUS "  - CUDA:                ${CUDA_VERSION}")
endif()
message(STATUS "")
message(STATUS "Compiler Flags:")
message(STATUS "  - CXX:                 ${CMAKE_CXX_FLAGS}")
message(STATUS "  - Architecture:        ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "=================================================================")
message(STATUS "")