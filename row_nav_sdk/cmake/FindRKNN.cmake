find_path(RKNN_INCLUDE_DIR
    NAMES rknn_api.h
    HINTS ${RKNN_ROOT} ENV RKNN_ROOT
    PATH_SUFFIXES include runtime/Linux/librknn_api/include)

find_library(RKNN_LIBRARY
    NAMES rknnrt
    HINTS ${RKNN_ROOT} ENV RKNN_ROOT
    PATH_SUFFIXES lib lib64 runtime/Linux/librknn_api/aarch64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RKNN REQUIRED_VARS RKNN_INCLUDE_DIR RKNN_LIBRARY)

if(RKNN_FOUND AND NOT TARGET RKNN::Runtime)
    add_library(RKNN::Runtime UNKNOWN IMPORTED)
    set_target_properties(RKNN::Runtime PROPERTIES
        IMPORTED_LOCATION "${RKNN_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RKNN_INCLUDE_DIR}")
endif()
