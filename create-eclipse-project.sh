#!/bin/bash
cd `dirname $0`

mkdir -p build
cd build
cmake -G"Eclipse CDT4 - Unix Makefiles" ../pinky/
