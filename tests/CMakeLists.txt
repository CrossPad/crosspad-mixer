# Host tests for the mixer engine: the engine alone, no LVGL, no app. Two
# binaries over the same tests: crosspad_mixer_tests takes render()'s scalar
# path, crosspad_mixer_tests_dsp the firmware's esp-dsp path through
# esp_dsp_stub/ (the same arithmetic as dsps_mulc_f32 + dsps_add_f32 on the
# ESP32-S3). Build and run:
#   cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
# Offline, point FetchContent at sources already on disk, e.g. crosspad-core's:
#   -DFETCHCONTENT_SOURCE_DIR_CATCH2=<dir> -DFETCHCONTENT_SOURCE_DIR_ARDUINOJSON=<dir>
# crosspad-core is used for its headers only; by default it is the sibling
# component of a platform checkout, else pass -DCROSSPAD_CORE_DIR=<path>.

cmake_minimum_required(VERSION 3.14)
project(crosspad-mixer-tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CROSSPAD_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../crosspad-core"
    CACHE PATH "crosspad-core checkout (headers only)")

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.0
)
FetchContent_MakeAvailable(Catch2)

# The engine saves its state as JSON
FetchContent_Declare(
    ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG        v7.3.0
)
FetchContent_MakeAvailable(ArduinoJson)

set(MIXER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

include(CTest)
include(Catch)

function(mixer_tests target)
    add_executable(${target}
        ${TEST_SOURCES}
        ${MIXER_DIR}/src/AudioMixerEngine.cpp
    )
    target_include_directories(${target} PRIVATE
        ${MIXER_DIR}/include
        ${CROSSPAD_CORE_DIR}/include
    )
    target_link_libraries(${target} PRIVATE
        Catch2::Catch2WithMain
        ArduinoJson
    )
endfunction()

mixer_tests(crosspad_mixer_tests)
catch_discover_tests(crosspad_mixer_tests)

mixer_tests(crosspad_mixer_tests_dsp)
target_compile_definitions(crosspad_mixer_tests_dsp PRIVATE CROSSPAD_USE_ESP_DSP=1)
target_include_directories(crosspad_mixer_tests_dsp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/esp_dsp_stub
)
catch_discover_tests(crosspad_mixer_tests_dsp TEST_PREFIX "esp-dsp.")
