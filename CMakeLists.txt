cmake_minimum_required(VERSION 2.8)

project(DePan)

set(DePan_MAJOR_VERSION 1)
set(DePan_MINOR_VERSION 11)
set(DePan_PATCH_VERSION 0)

set(DePan_VERSION ${DePan_MAJOR_VERSION}.${DePan_MINOR_VERSION}.${DePan_PATCH_VERSION})

find_package(PkgConfig)
pkg_check_modules(AVXSYNTH REQUIRED avxsynth)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

include_directories(${AVXSYNTH_INCLUDEDIR} ${FFTW3F_INCLUDEDIR})


add_library(DePanEstimate MODULE
    depanestimate.cpp
    depanio.cpp
    estimate_fftw.cpp
    info.cpp)
target_link_libraries(DePanEstimate ${FFTW3F_LIBRARIES})

add_library(DePan MODULE
    compensate.cpp
    depanio.cpp
    info.cpp
    interface.cpp
    interpolate.cpp
    scenes.cpp
    stabilize.cpp
    transform.cpp)
target_link_libraries(DePan ${FFTW3F_LIBRARIES})

install(TARGETS DePanEstimate LIBRARY DESTINATION lib/avxsynth)
install(TARGETS DePan LIBRARY DESTINATION lib/avxsynth)
