cmake_minimum_required(VERSION 2.8)

project(depanestimate)

find_package(PkgConfig)
pkg_check_modules(AVXSYNTH REQUIRED avxsynth)

include_directories(${AVXSYNTH_INCLUDEDIR})

add_library(DePanEstimate MODULE depanestimate.cpp depanio.cpp estimate_fftw.cpp info.cpp)

INSTALL(TARGETS DePanEstimate LIBRARY DESTINATION lib/avxsynth)
