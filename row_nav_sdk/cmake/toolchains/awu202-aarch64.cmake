set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(NOT DEFINED ENV{AWU202_TOOLCHAIN})
    message(FATAL_ERROR "AWU202_TOOLCHAIN must point to the extracted aarch64--glibc toolchain")
endif()

set(AWU202_TOOLCHAIN "$ENV{AWU202_TOOLCHAIN}")
set(CMAKE_C_COMPILER "${AWU202_TOOLCHAIN}/bin/aarch64-linux-gcc")
set(CMAKE_CXX_COMPILER "${AWU202_TOOLCHAIN}/bin/aarch64-linux-g++")
set(CMAKE_SYSROOT "${AWU202_TOOLCHAIN}/aarch64-buildroot-linux-gnu/sysroot")

# The OpenCV overlay is copied from the target board. Its shared libraries may
# have dependencies outside the minimal compiler sysroot; the board resolves
# those dependencies when the resulting executable is deployed.
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--allow-shlib-undefined")

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# BOTH keeps target sysroot lookup while allowing explicitly supplied dependency
# roots such as the repository-local RKNN Runtime bundle.
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
