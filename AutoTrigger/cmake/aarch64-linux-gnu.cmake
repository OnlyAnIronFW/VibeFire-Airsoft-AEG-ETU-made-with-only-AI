set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# RKNN SDK for RK356X (installed manually at /opt/rknpu2)
set(RKNN_SDK_ROOT "/opt/rknpu2" CACHE PATH "RKNN SDK root directory")
set(CMAKE_PREFIX_PATH "${RKNN_SDK_ROOT}/runtime/RK356X/Linux/librknn_api/aarch64")
