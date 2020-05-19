#include(FindPackageHandleStandardArgs.cmake)

find_path(Sophus_INCLUDE_DIR scso3.h se2.h se3.h sim3.h so2.h so3.h
    PATHS
    /home/hb/devel/include/sophus
    /home/hb/devel/include/
    /usr/include
    /usr/local/include)

find_library(Sophus_LIB Sophus
    PATHS
    /home/hb/devel/lib
    /usr/local/lib
    /usr/lib)

find_package_handle_standard_args(Sophus
 DEFAULT_MSG
  Sophus_INCLUDE_DIR
  Sophus_LIB
)
include(FindPackageHandleStandardArgs)
mark_as_advanced(Sophus_LIB Sophus_INCLUDE_DIR)

if(Sophus_FOUND)
  set(Sophus_LIBRARIES    ${Sophus_LIB})
  set(Sophus_INCLUDE_DIRS ${Sophus_INCLUDE_DIR})
endif()