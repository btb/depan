cmake_minimum_required(VERSION 2.8)

project(depan)

find_package(PkgConfig)
pkg_check_modules(AVXSYNTH REQUIRED avxsynth)

include_directories(${AVXSYNTH_INCLUDEDIR})

add_library(DePanEstimate MODULE
    depanestimate.cpp
    depanio.cpp
    estimate_fftw.cpp
    info.cpp)

add_library(DePan MODULE
    compensate.cpp
    depanio.cpp
    info.cpp
    interface.cpp
    interpolate.cpp
    scenes.cpp
    stabilize.cpp
    transform.cpp)

INSTALL(TARGETS DePanEstimate LIBRARY DESTINATION lib/avxsynth)
INSTALL(TARGETS DePan LIBRARY DESTINATION lib/avxsynth)
