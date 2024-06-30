set(RELOCATED_HOST_DIR $ENV{AIQ_BUILD_HOST_DIR})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)

set(CMAKE_SKIP_RPATH TRUE)

set(CMAKE_C_FLAGS_DEBUG "" CACHE STRING "Debug CFLAGS")
set(CMAKE_CXX_FLAGS_DEBUG "" CACHE STRING "Debug CXXFLAGS")
set(CMAKE_C_FLAGS_RELEASE " -DNDEBUG" CACHE STRING "Release CFLAGS")
set(CMAKE_CXX_FLAGS_RELEASE " -DNDEBUG" CACHE STRING "Release CXXFLAGS")

set(CMAKE_C_FLAGS "-D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -Os" CACHE STRING "Buildroot CFLAGS")
set(CMAKE_CXX_FLAGS "-D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -Os" CACHE STRING "Buildroot CXXFLAGS")
set(CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "Buildroot LDFLAGS for executables")

set(CMAKE_INSTALL_SO_NO_EXE 0)

#set(CMAKE_PROGRAM_PATH "${RELOCATED_HOST_DIR}/bin")
set(CMAKE_SYSROOT "${RELOCATED_HOST_DIR}/arm-buildroot-linux-gnueabihf/sysroot")
set(CMAKE_FIND_ROOT_PATH "${RELOCATED_HOST_DIR}/arm-buildroot-linux-gnueabihf/sysroot")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${RELOCATED_HOST_DIR}/arm-buildroot-linux-gnueabihf/sysroot")

set(CMAKE_C_COMPILER "${RELOCATED_HOST_DIR}/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${RELOCATED_HOST_DIR}/bin/arm-linux-gnueabihf-g++")
