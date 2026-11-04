cmake_minimum_required(VERSION 3.10)

# Module version
project(SoapyPerseus VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Recherche de SoapySDR
find_package(SoapySDR CONFIG REQUIRED)

# 2. Recherche de la bibliothèque libperseus-sdr
find_library(PERSEUS_LIBRARY NAMES perseus-sdr REQUIRED)
find_path(PERSEUS_INCLUDE_DIR NAMES perseus-sdr.h REQUIRED)

include_directories(${PERSEUS_INCLUDE_DIR})

# 3. Utilisation de la macro système (comme SoapyRTLSDR)
# Elle va détecter "SoapyPerseus_VERSION" (0.1.0) et l'encoder dans le .so
SOAPY_SDR_MODULE_UTIL(
    TARGET perseusSupport
    SOURCES
        SoapyPerseus.cpp
        Registration.cpp
    LIBRARIES
        ${SoapySDR_LIBRARIES}
        ${PERSEUS_LIBRARY}
)

# ==============================================================================
# RÈGLE DE DÉSINSTALLATION : make uninstall
# ==============================================================================
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
endif()
