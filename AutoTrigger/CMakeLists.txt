cmake_minimum_required(VERSION 3.16)
project(autotrigger VERSION 1.0.0 LANGUAGES CXX)

# ---- Build configuration ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra)
endif()

# ---- Dual-target: mock (x86_64 dev) vs real (aarch64 RK3566) ----
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Target: aarch64 — enabling real hardware drivers")
else()
  message(STATUS "Target: ${CMAKE_SYSTEM_PROCESSOR} — enabling mock mode")
  add_compile_definitions(MOCK_MODE)
endif()

# ---- Dependencies ----

# GoogleTest (FetchContent)
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
# Only populate when tests are enabled
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  include(GoogleTest)
endif()

# Eigen3 (header-only, fetched via FetchContent for portability)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)
set(EIGEN3_INCLUDE_DIR ${eigen_SOURCE_DIR})
message(STATUS "Eigen3 fetched: ${EIGEN3_INCLUDE_DIR}")

# OpenCV (REQUIRED for production; set AUTOTRIGGER_SKIP_DEPS=ON for scaffolding tests)
option(AUTOTRIGGER_SKIP_DEPS "Skip optional dependency checks for quick dev builds" OFF)
if(AUTOTRIGGER_SKIP_DEPS)
  find_package(OpenCV QUIET COMPONENTS core imgproc calib3d)
  if(NOT OpenCV_FOUND)
    message(WARNING "OpenCV not found — build will succeed but autotrigger_core is incomplete")
  endif()
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
endif()
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
endif()

# SDL2
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
  message(STATUS "SDL2 not found — display features will be disabled at link time")
else()
  message(STATUS "Found SDL2: ${SDL2_INCLUDE_DIRS}")
endif()

# libgpiod (ARM64 only, optional)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(LIBGPIOD libgpiod)
  endif()
  if(NOT LIBGPIOD_FOUND)
    message(WARNING "libgpiod not found — GPIO features will be disabled")
  endif()
endif()

# ---- autotrigger_core (static library) ----
# Exclude main.cpp — library must not contain a main() entry point
# that would conflict with gtest_main when linking test executables.
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(FILTER CORE_SOURCES EXCLUDE REGEX "main\\.cpp$")
file(GLOB_RECURSE CORE_HEADERS "include/autotrigger/*.h" "include/autotrigger/hal/*.h")

add_library(autotrigger_core STATIC ${CORE_SOURCES} ${CORE_HEADERS})
target_include_directories(autotrigger_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(OpenCV_FOUND)
  target_link_libraries(autotrigger_core
    PUBLIC
      OpenCV::core
      OpenCV::imgproc
      OpenCV::calib3d
  )
endif()
target_include_directories(autotrigger_core SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

if(SDL2_FOUND)
  target_link_libraries(autotrigger_core PUBLIC ${SDL2_LIBRARIES})
  target_include_directories(autotrigger_core SYSTEM PUBLIC ${SDL2_INCLUDE_DIRS})
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" AND LIBGPIOD_FOUND)
  target_link_libraries(autotrigger_core PUBLIC ${LIBGPIOD_LIBRARIES})
  target_include_directories(autotrigger_core SYSTEM PUBLIC ${LIBGPIOD_INCLUDE_DIRS})
endif()

# ---- autotrigger (executable) ----
add_executable(autotrigger src/main.cpp)
target_link_libraries(autotrigger PRIVATE autotrigger_core)

# ---- Tests ----
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
