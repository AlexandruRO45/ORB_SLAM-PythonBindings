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

        print(f"Building extension for Python {sys.version}")
        print(f"Extension output directory: {extdir}")

        # Define the arguments to pass to CMake
        cmake_args = [
            # Tell CMake where to put the resulting library
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            # Pass the Python executable to CMake for discovery
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            # Set the build type (Release, Debug, etc.)
            f"-DCMAKE_BUILD_TYPE={build_type}",
            # Pass header paths (though modern find_package(Python) is often sufficient)
            f"-DPYTHON_INCLUDE_DIR={python_include_dir}",
            f"-DNUMPY_INCLUDE_DIR={numpy_include_dir}",
            # Suppress unnecessary CMake developer warnings
            "-Wno-dev",
        ]
        
        # Check for a Pangolin_DIR environment variable to locate Pangolin
        # USE export Pangolin_DIR="<path_to_user>/Pangolin-0.9.1/build" if not system installed
        pangolin_dir = os.environ.get("Pangolin_DIR")
        if pangolin_dir:
            print(f"Found Pangolin_DIR environment variable: {pangolin_dir}")
            cmake_args.append(f"-DCMAKE_PREFIX_PATH={pangolin_dir}")

        # Allow user to pass extra CMake arguments via environment variable
        if "CMAKE_ARGS" in os.environ:
            cmake_args += os.environ["CMAKE_ARGS"].split()

        # Create the temporary build directory
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        # 1. Configure the project with CMake
        print(f"Configuring CMake project with: {' '.join(cmake_args)}")
        subprocess.check_call(
            ["cmake", str(ext.sourcedir)] + cmake_args, cwd=build_temp
        )

        # Allow user to override the number of parallel build jobs
        # build_jobs = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        # if not build_jobs:
        #     build_jobs = str(os.cpu_count() or 8) # Default to number of CPUs
        build_jobs = str(8)

        # 2. Build the project
        print(f"Building project with {build_jobs} parallel jobs")
        subprocess.check_call(
            ["cmake", "--build", ".", "--parallel", build_jobs],
            cwd=build_temp
        )

# --- Main setup() function ---
setup(
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    ext_modules=[CMakeExtension("orbslam3._core", sourcedir=".")], 
    cmdclass={"build_ext": CMakeBuild},
    package_data={
        'orbslam3': ['*.so', '*.pyd', '*.dylib'], 
    },
    include_package_data=True,
    zip_safe=False,
)