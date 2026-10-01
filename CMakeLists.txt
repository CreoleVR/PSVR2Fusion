cmake_minimum_required(VERSION 3.19)
project(psvr2fusion VERSION 1.0.0 LANGUAGES C CXX)

if(NOT WIN32)
  message(FATAL_ERROR "PSVR2Fusion targets Windows SteamVR only.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OPENVR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/openvr")
set(MINHOOK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/minhook")
set(DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/driver/psvr2fusion")
set(OUTPUT_DIR "${DRIVER_ROOT}/bin/win64")

add_library(minhook STATIC
  "${MINHOOK_ROOT}/src/buffer.c"
  "${MINHOOK_ROOT}/src/hook.c"
  "${MINHOOK_ROOT}/src/trampoline.c"
  "${MINHOOK_ROOT}/src/hde/hde64.c")
target_include_directories(minhook PUBLIC "${MINHOOK_ROOT}/include")
if(MSVC)
  target_compile_options(minhook PRIVATE /W0)
endif()

add_library(driver_psvr2fusion SHARED
  src/driver.cpp
  src/imu_source.cpp
  src/log.cpp
  src/room_setup.cpp
  src/settings.cpp)
set_target_properties(driver_psvr2fusion PROPERTIES
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIR}")
target_include_directories(driver_psvr2fusion PRIVATE src)
target_include_directories(driver_psvr2fusion SYSTEM PRIVATE "${OPENVR_ROOT}/headers")
target_compile_definitions(driver_psvr2fusion PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
target_link_libraries(driver_psvr2fusion PRIVATE minhook)
if(MSVC)
  target_compile_options(driver_psvr2fusion PRIVATE /W4 /utf-8)
  # No VC++ redistributable needed on the user's PC.
  set_property(TARGET driver_psvr2fusion minhook PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

enable_testing()
add_executable(psvr2fusion_tests tests/tests.cpp)
target_include_directories(psvr2fusion_tests PRIVATE src)
if(MSVC)
  target_compile_options(psvr2fusion_tests PRIVATE /W4 /utf-8)
endif()
add_test(NAME psvr2fusion_tests COMMAND psvr2fusion_tests)
