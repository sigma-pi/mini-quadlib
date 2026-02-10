"""
Mini-QuadLib Python Bindings - Complete C API Coverage

A comprehensive Python wrapper for the mini-quadlib C library providing 
almost all C API functions with minimal abstraction.
"""

# Import everything from core
from .core import *

__version__ = "0.2.0"
__author__ = "Chengyu Yang"
__email__ = "chengyuy520@gmail.com"

# Re-export everything for convenience
# (Everything is already imported via "from .core import *")