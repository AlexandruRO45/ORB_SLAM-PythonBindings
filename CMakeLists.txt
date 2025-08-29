cmake_minimum_required(VERSION 3.4...3.18)
project(ORB_SLAM_PYTHON_BINDING)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if(DEFINED ENV{Pangolin_DIR})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{Pangolin_DIR}")
  message(STATUS "Using Pangolin_DIR from environment: $ENV{Pangolin_DIR}")
endif()
find_package(Pangolin REQUIRED)
message(STATUS "Found Pangolin ${PANGOLIN_INCLUDE_DIR} ${PANGOLIN_LIBRARY}")

find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
find_package(NumPy REQUIRED)
if(NOT NumPy_INCLUDE_DIR AND DEFINED NUMPY_INCLUDE_DIR)
  set(NumPy_INCLUDE_DIR ${NUMPY_INCLUDE_DIR})
  message(STATUS "Using NumPy include directory from setup.py: ${NumPy_INCLUDE_DIR}")
endif()

find_package(Boost REQUIRED COMPONENTS python thread system serialization)
message(STATUS "Found Boost.Python component: ${Boost_LIBRARIES}")
find_package(OpenCV REQUIRED)
message(STATUS "Found OPENCV ${OpenCV_LIBS}")

# --- Set Global Compiler Flags ---
set(BASE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-maybe-uninitialized -Wno-deprecated-declarations")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
  set(ARCH_FLAGS "-O3 -march=armv8-a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set(ARCH_FLAGS "-O3 -mfpu=neon -march=armv7-a")
else() # x86_64
  set(ARCH_FLAGS "-O3 -mmmx -msse -msse2 -msse3 -mssse3")
endif()

# Append these flags to the default CMake flags
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} ${BASE_CXX_FLAGS} ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BASE_CXX_FLAGS} ${ARCH_FLAGS}")

# --- Add Core ORB_SLAM3 Library ---
add_subdirectory(orbslam3)

# --- Create the Python Module ---
set(PYTHON_MODULE_NAME orbslam3_python) 
add_library(${PYTHON_MODULE_NAME} SHARED
  src/ORBSlamPython.cpp
  src/pyboostcvconverter/pyboost_cv3_converter.cpp
)

# Set properties to create the correct python module filename (e.g., "orbslam3.so")
# We set OUTPUT_NAME to the desired import name and remove the "lib" prefix.
set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES
  PREFIX ""
  OUTPUT_NAME "orbslam3"
)

# --- Configure Python Module Target ---

# Add include directories required to BUILD the python module
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

# --- Output Paths ---
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)