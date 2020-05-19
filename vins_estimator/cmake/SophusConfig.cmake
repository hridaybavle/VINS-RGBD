find_library(sophus_LIBRARY Sophus
    PATHS "/home/hb/devel/lib"
    PATHS "/usr/lib"
 
)
find_path(sophus_INCLUDE_DIR sophus
    PATHS "/home/hb/devel/include"
)
set(sophus_LIBS ${sophus_LIBRARY})
set(sophus_LIBRARIES ${sophus_LIBRARY})
set(sophus_INCLUDE_DIRS ${sophus_INCLUDE_DIR})