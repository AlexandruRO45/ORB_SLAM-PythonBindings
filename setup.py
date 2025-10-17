"""
Setup script for ORB-SLAM3 Enhanced Python Bindings

This uses setuptools with a custom CMake build extension to compile
the C++ ORB-SLAM3 wrapper with enhanced features including:
- Hardware detection (CPU/GPU)
- Power mode management (HIGH/LOW)
- Comprehensive performance metrics
- Confidence-based outputs
"""

import os
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup, find_packages
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """
    A custom setuptools Extension class for CMake-based projects.
    This is a marker class that holds the location of the CMakeLists.txt file.
    """
    def __init__(self, name: str, sourcedir: str = "", **kwargs):
        super().__init__(name, sources=[], **kwargs)
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """
    Custom build_ext command to drive the CMake build process.
    Supports both enhanced and legacy bindings.
    """
    def build_extension(self, ext: CMakeExtension):
        # Import numpy here, only when building, to ensure it's installed
        import numpy as np
        import sysconfig

        # The final destination for the compiled extension library.
        # self.get_ext_fullpath(ext.name) returns the path to the installed
        # module, e.g., build/lib.linux-x86_64-3.10/orbslam3.so
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()

        # Get Python and NumPy header paths
        python_include_dir = sysconfig.get_path("include")
        numpy_include_dir = np.get_include()

        # Allow user to override build type with an environment variable
        build_type = os.environ.get("CMAKE_BUILD_TYPE", "Release")
        
        # Check if enhanced bindings should be built
        build_enhanced = os.environ.get("BUILD_ENHANCED_BINDINGS", "ON")
        enable_cuda = os.environ.get("ENABLE_CUDA", "OFF")

        print(f"Building extension for Python {sys.version}")
        print(f"Extension output directory: {extdir}")
        print(f"Enhanced bindings: {build_enhanced}")
        print(f"CUDA support: {enable_cuda}")

        # Define the arguments to pass to CMake
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={build_type}",
            f"-DPYTHON_INCLUDE_DIR={python_include_dir}",
            f"-DNUMPY_INCLUDE_DIR={numpy_include_dir}",
            f"-DBUILD_ENHANCED_BINDINGS={build_enhanced}",
            f"-DENABLE_CUDA={enable_cuda}",
            "-Wno-dev",
        ]
        prefix_paths = []

        # If CUDA is enabled, add its paths to the prefix list
        if enable_cuda == "ON":
            cuda_dir = os.environ.get("CUDA_DIR") 
            if cuda_dir:
                print(f"Found CUDA at: {cuda_dir}")
                libcudacxx_path = os.path.join(cuda_dir, "lib64", "cmake", "libcudacxx")
                print(f"Set libcudacxx environment variable at: {libcudacxx_path}")
                prefix_paths.append(libcudacxx_path)
            else:
                print("WARNING: ENABLE_CUDA is ON, but CUDA_DIR environment variable is not set.")

        # Add Pangolin build directory to the prefix list
        # USE: export Pangolin_DIR="<path_to_user>/Pangolin/build"
        pangolin_dir = os.environ.get("Pangolin_DIR") 
        if pangolin_dir:
            print(f"Found Pangolin_DIR environment variable: {pangolin_dir}")
            prefix_paths.append(pangolin_dir)
        else:
            print("WARNING: Pangolin_DIR not set. Pangolin might not be found by CMake.")
        
        # Join all collected paths and add the final CMAKE_PREFIX_PATH argument
        if prefix_paths:
            joined_paths = ";".join(prefix_paths)
            print(f"Amending CMAKE_PREFIX_PATH with: {joined_paths}")
            cmake_args.append(f'-DCMAKE_PREFIX_PATH={joined_paths}')

        # Allow user to pass extra CMake arguments via environment variable
        if "CMAKE_ARGS" in os.environ:
            cmake_args += os.environ["CMAKE_ARGS"].split()

        # Create the temporary build directory
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        # 1. Configure the project with CMake
        print(f"\n{'='*70}")
        print("CONFIGURING CMAKE PROJECT")
        print(f"{'='*70}")
        print(f"CMake args: {' '.join(cmake_args)}")
        print(f"{'='*70}\n")
        
        try:
            subprocess.check_call(
                ["cmake", str(ext.sourcedir)] + cmake_args, 
                cwd=build_temp
            )
        except subprocess.CalledProcessError as e:
            print(f"\n{'='*70}")
            print("CMAKE CONFIGURATION FAILED")
            print(f"{'='*70}")
            print("Possible issues:")
            print("  1. CMake version too old (requires >= 3.4)")
            print("  2. Missing dependencies (Pangolin, OpenCV, Eigen3, Boost)")
            print("  3. ORB_SLAM3 not found (set ORB_SLAM3_DIR environment variable)")
            print(f"{'='*70}\n")
            raise

        # Allow user to override the number of parallel build jobs
        build_jobs = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        if not build_jobs:
            build_jobs = str(os.cpu_count() or 8) # Default to number of CPUs

        # 2. Build the project
        print(f"\n{'='*70}")
        print(f"BUILDING PROJECT WITH {build_jobs} PARALLEL JOBS")
        print(f"{'='*70}\n")
        
        try:
            subprocess.check_call(
                ["cmake", "--build", ".", "--parallel", build_jobs],
                cwd=build_temp
            )
        except subprocess.CalledProcessError as e:
            print(f"\n{'='*70}")
            print("CMAKE BUILD FAILED")
            print(f"{'='*70}")
            print("Possible issues:")
            print("  1. Compilation errors (check C++ code)")
            print("  2. Missing libraries at link time")
            print("  3. Insufficient memory (try reducing parallel jobs)")
            print(f"{'='*70}\n")
            raise

        print(f"\n{'='*70}")
        print("BUILD COMPLETED SUCCESSFULLY")
        print(f"Extension installed to: {extdir}")
        print(f"{'='*70}\n")


# --- Main setup() function ---
setup(
    # Package discovery
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    
    # CMake extension 
    ext_modules=[
        CMakeExtension("orbslam3._core", sourcedir=".")
    ],
    
    # Custom build command
    cmdclass={"build_ext": CMakeBuild},
    
    # Include compiled libraries
    package_data={
        'orbslam3': [
            '*.so',           # Linux
            '*.pyd',          # Windows
            '*.dylib',        # macOS
            '_version.py',    # Version info
        ],
    },
    
    include_package_data=True,
    zip_safe=False,
)