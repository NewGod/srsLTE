
message(STATUS "FINDING YUNSDR.")
if(NOT YUNSDR_FOUND)
    #pkg_check_modules (YUNSDR_PKG YunSDR)

  find_path(YUNSDR_INCLUDE_DIRS 
    NAMES riffa.h
    PATHS
          /usr/local/include/
  )

  find_library(YUNSDR_LIBRARIES 
    NAMES riffa
    PATHS
          /usr/local/lib
  )


if(YUNSDR_INCLUDE_DIRS AND YUNSDR_LIBRARIES)
  set(YUNSDR_FOUND TRUE CACHE INTERNAL "libriffa found")
  message(STATUS "Found libriffa: ${YUNSDR_INCLUDE_DIRS}, ${YUNSDR_LIBRARIES}")
else(YUNSDR_INCLUDE_DIRS AND YUNSDR_LIBRARIES)
  set(YUNSDR_FOUND FALSE CACHE INTERNAL "libriffa found")
  message(STATUS "libriffa not found.")
endif(YUNSDR_INCLUDE_DIRS AND YUNSDR_LIBRARIES)

mark_as_advanced(YUNSDR_LIBRARIES YUNSDR_INCLUDE_DIRS)

endif(NOT YUNSDR_FOUND)
