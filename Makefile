include $(shell rospack find mk)/cmake.mk

# Clang is a good compiler to use during development due to its faster compile
# times and more readable output.
# C_compiler=/usr/bin/clang
# CXX_compiler=/usr/bin/clang++

# GCC is better for release mode due to the speed of its output, and its support
# for OpenMP.
C_compiler=/usr/bin/gcc
CXX_compiler=/usr/bin/g++

#acceptable buildTypes: Release/Debug/Profile
# buildType=Release
buildType=Debug

ifeq ($(buildType),Debug)
	buildDir=build_debug
else
	buildDir=build
endif

.SILENT:

all: build

cmake: CMakeLists.txt
	cd $(buildDir) && cmake -DCMAKE_BUILD_TYPE=$(buildType) -DCMAKE_CXX_COMPILER=$(CXX_compiler) -DCMAKE_C_COMPILER=$(C_compiler) ..

build:	cmake
	$(MAKE) --no-print-directory -C $(buildDir)

clean:
	$(MAKE) --no-print-directory -C $(buildDir) clean

cleanup_cache:
	cd $(buildDir) && rm -rf *

