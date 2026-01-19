#!/bin/bash
rm -rf build
cmake -B build -DBUILD_TESTS=ON && cmake --build build
./build/test_basic
