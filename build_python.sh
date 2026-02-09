#!/bin/bash
# Fixed build script with better testing

set -e

echo "Building mini-quadlib Python package..."

# Ensure we're in the right directory
cd "$(dirname "$0")"

# Build C library as shared library
echo "Building C shared library..."
rm -rf build
mkdir build
cd build

cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)

# Verify the shared library was created
if [ -f "libmini_quadlib.so" ]; then
  echo "✓ Shared library built successfully"
  echo "Checking symbols in library..."
  nm -D libmini_quadlib.so | grep -E "(quadlib_version|vector3_add|geometric_control)" | head -5
else
  echo "✗ Failed to build libmini_quadlib.so"
  exit 1
fi

cd ..

# Build Python package
echo "Building Python package..."
cd python

# Create local README if it doesn't exist
if [ ! -f "README.md" ]; then
  echo "Creating local README.md..."
  cat > README.md << 'EOF'
# Mini QuadLib - Python Bindings

Python bindings for mini-quadlib: A C library for quadrotor control systems.

## Quick Start

```python
import mini_quadlib as mql
import numpy as np

# Get library version
print(mql.get_version())

# Quaternion operations
q = mql.Quaternion(0.707, 0.0, 0.0, 0.707)
q_norm = q.normalize()
q_conj = q.conjugate()

# Coordinate transforms
ned = np.array([1.0, 2.0, 3.0])
enu = mql.ned_to_enu(ned)
```
EOF
fi

# Clean previous builds
rm -rf build/ dist/ *.egg-info/

# Install build tools if needed
pip install --upgrade build twine

# Install in development mode
echo "Installing in development mode..."
pip install -e .

# Test the installation
echo "Testing installation..."
python -c "
import mini_quadlib as mql
import numpy as np
print('✓ Import successful')

# Test version
version = mql.get_version()
print(f'✓ Version: {version}')

# Test Quaternion
q = mql.Quaternion(1.0, 0.0, 0.0, 0.0)
q_norm = q.normalize()
print(f'✓ Quaternion test: {q} -> normalized: {q_norm}')

# Test coordinate transform
ned = np.array([1.0, 2.0, 3.0])
enu = mql.ned_to_enu(ned)
print(f'✓ NED to ENU: {ned} -> {enu}')

print('✓ Basic tests passed!')
"

echo ""
echo "✓ Build complete!"
echo "✓ Package installed in development mode"
echo ""
echo "Available C functions in library:"
nm -D ../build/libmini_quadlib.so 2>/dev/null | grep -E "T (vector3|quaternion|geometric)" | head -10 || echo "Could not list symbols"
echo ""
echo "Test with: python -c 'import mini_quadlib as mql; print(mql.get_version())'"