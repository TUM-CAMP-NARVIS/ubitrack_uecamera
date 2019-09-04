project(ubitrack_component_core)

cmake_minimum_required(VERSION 3.4.0)

include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)


add_subdirectory(src/uecamera)


ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml"  "doc/utql/*/*/*/*.xml")

