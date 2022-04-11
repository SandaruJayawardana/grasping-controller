#!/bin/bash
echo "Build the preprocessing files"

cd build

cmake -DOpen3D_ROOT=${HOME}/open3d_install ..
make -j 4
