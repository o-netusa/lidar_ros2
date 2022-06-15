cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(DownloadProject)

# ============================================================
#                        cppbase
# ============================================================
#
message("==> Configuring cppbase")
set(CPPBASE_BUILD_GTEST OFF)
download_project(
  PROJ     cppbase
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/cppbase/cppbase.tar.gz
  URL_HASH MD5=cc46d3d7ceba5cde9127d03fc4a76446
  QUIET
)
add_subdirectory(${cppbase_SOURCE_DIR} ${cppbase_BINARY_DIR})
set(CPPBASE_INCLUDE_DIR ${cppbase_SOURCE_DIR} CACHE STRING "")

# ============================================================
#                        lidardevice
# ============================================================
#
message("==> Configuring lidardevice")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
message("==> Configuring lidardevice x86_64")
download_project(
  PROJ     lidardevice
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/lidardevice/LidarDevice-1.0.0-x86_64_Linux.tar.gz
  URL_HASH MD5=b3beefb28032ba95e085ea3d42819aa9   
  QUIET
)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)")
message("==> Configuring lidardevice aarch")
download_project(
  PROJ     lidardevice
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/lidardevice/LidarDevice-1.0.0-aarch64_Linux.tar.gz
  URL_HASH MD5=749866690a24b6ff095d4c007b003bd2 
  QUIET
)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)")
message("==> Configuring lidardevice arm")
download_project(
  PROJ     lidardevice
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/lidardevice/LidarDevice-1.0.0-arm_Linux.tar.gz
  URL_HASH MD5=7754bea02e4f3dd51b9a46df6a7bc595  
  QUIET
)
else ()
message("The architecture is not support!")
endif ()

add_library(lidardevice INTERFACE)
target_include_directories(lidardevice INTERFACE ${lidardevice_SOURCE_DIR}/include)
target_link_directories(lidardevice INTERFACE ${lidardevice_SOURCE_DIR}/lib)
target_link_libraries(lidardevice INTERFACE LidarDevice eigen sigslot stduuid)
set(lidardevice_SOURCE_DIR ${lidardevice_SOURCE_DIR} CACHE STRING "")
