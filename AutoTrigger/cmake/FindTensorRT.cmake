# cmake/FindTensorRT.cmake
# Locate NVIDIA TensorRT SDK and define TensorRT::tensorrt imported target.
#
# Cache variables (set before find_package):
#   TensorRT_ROOT   - root directory of TensorRT installation
#
# Defined after find_package:
#   TensorRT_FOUND          - TRUE if found
#   TensorRT_INCLUDE_DIRS   - NvInfer.h directory
#   TensorRT_LIBRARIES      - nvinfer library path
#   TensorRT::tensorrt      - imported target

# ---- Search paths ----
set(_TensorRT_SEARCH_PATHS)
if(TensorRT_ROOT)
  list(APPEND _TensorRT_SEARCH_PATHS ${TensorRT_ROOT})
endif()

# Common install locations
list(APPEND _TensorRT_SEARCH_PATHS
  /usr/local/TensorRT-10.7
  /usr/local/TensorRT-10.6
  /usr/local/TensorRT-10.5
  /usr/local/TensorRT-10.4
  /usr/local/TensorRT-10.3
  /usr/local/TensorRT-10.2
  /usr/local/TensorRT-10.1
  /usr/local/TensorRT-10.0
  /usr/local/TensorRT-8.6
  /usr/local/TensorRT-8.5
  /usr/local/TensorRT
  /usr
)

# ---- Find include directory ----
find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES include
  DOC "TensorRT include directory"
)

# ---- Find library ----
find_library(TensorRT_LIBRARY
  NAMES nvinfer
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES lib lib64
  DOC "TensorRT nvinfer library"
)

# ---- Extract version from NvInfer.h ----
if(TensorRT_INCLUDE_DIR AND EXISTS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _TRT_MAJOR
       REGEX "^#define NV_TENSORRT_MAJOR [0-9]+")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _TRT_MINOR
       REGEX "^#define NV_TENSORRT_MINOR [0-9]+")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _TRT_PATCH
       REGEX "^#define NV_TENSORRT_PATCH [0-9]+")
  string(REGEX MATCH "[0-9]+" _TRT_MAJOR "${_TRT_MAJOR}")
  string(REGEX MATCH "[0-9]+" _TRT_MINOR "${_TRT_MINOR}")
  string(REGEX MATCH "[0-9]+" _TRT_PATCH "${_TRT_PATCH}")
  set(TensorRT_VERSION "${_TRT_MAJOR}.${_TRT_MINOR}.${_TRT_PATCH}")
elseif(TensorRT_INCLUDE_DIR AND EXISTS "${TensorRT_INCLUDE_DIR}/NvInfer.h")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInfer.h" _TRT_MAJOR
       REGEX "^#define NV_TENSORRT_MAJOR [0-9]+")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInfer.h" _TRT_MINOR
       REGEX "^#define NV_TENSORRT_MINOR [0-9]+")
  string(REGEX MATCH "[0-9]+" _TRT_MAJOR "${_TRT_MAJOR}")
  string(REGEX MATCH "[0-9]+" _TRT_MINOR "${_TRT_MINOR}")
  set(TensorRT_VERSION "${_TRT_MAJOR}.${_TRT_MINOR}.0")
endif()

# ---- Standard find ----
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
  REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_LIBRARY
  VERSION_VAR   TensorRT_VERSION
)

# ---- Create imported target ----
if(TensorRT_FOUND AND NOT TARGET TensorRT::tensorrt)
  set(TensorRT_INCLUDE_DIRS ${TensorRT_INCLUDE_DIR})
  set(TensorRT_LIBRARIES    ${TensorRT_LIBRARY})

  add_library(TensorRT::tensorrt UNKNOWN IMPORTED)
  set_target_properties(TensorRT::tensorrt PROPERTIES
    IMPORTED_LOCATION "${TensorRT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
  )
endif()
