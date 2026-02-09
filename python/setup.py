from setuptools import setup
from setuptools.command.build_py import build_py
import subprocess
import os

class BuildWithCMake(build_py):
    """Custom build command that ensures C library is built first"""
    
    def run(self):
        self.build_c_library()
        super().run()
    
    def build_c_library(self):
        """Build the C library using CMake if not already built"""
        root_dir = os.path.dirname(os.path.abspath(__file__))
        build_dir = os.path.join(root_dir, '..', 'build')
        repo_root = os.path.join(root_dir, '..')

        # Skip if library already exists
        if os.path.exists(os.path.join(build_dir, 'libmini_quadlib.so')):
            print("C library already built, skipping cmake.")
            return

        # Skip if we're not in the real repo (e.g. sdist temp dir)
        if not os.path.exists(os.path.join(repo_root, 'CMakeLists.txt')):
            print("Warning: CMakeLists.txt not found, skipping C build.")
            print("Build the C library first with: ./build_python.sh")
            return

        print("Building C library...")
        original_dir = os.getcwd()
        try:
            os.chdir(repo_root)
            
            if not os.path.exists('build'):
                os.makedirs('build')
            
            os.chdir('build')
            subprocess.run(['cmake', '..', '-DBUILD_PYTHON=ON'], check=True)
            subprocess.run(['make', '-j4'], check=True)
        finally:
            os.chdir(original_dir)

# All project configuration is now in pyproject.toml
# This setup.py only handles the custom CMake build
setup(
    cmdclass={'build_py': BuildWithCMake},
)