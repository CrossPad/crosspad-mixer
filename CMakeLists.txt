# CrossPad Mixer — ESP-IDF Component CMakeLists.txt
#
# All sources live in src/ and compile on every platform. The single
# MixerApp.cpp uses __has_include to pick the right App header
# (pc_stubs/PcApp.hpp on PC, crosspad-gui/app/App.hpp on IDF).

file(GLOB MIXER_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

idf_component_register(
    SRCS ${MIXER_SRCS}
    INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    REQUIRES
        crosspad-core
        crosspad-gui
        lvgl
        ArduinoJson
    PRIV_REQUIRES
        espressif__esp-dsp
)

# render()'s esp-dsp kernels (src/AudioMixerEngine.cpp) where esp-dsp is built.
if(IDF_TARGET STREQUAL "esp32s3")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CROSSPAD_USE_ESP_DSP=1)
endif()

# WHOLE_ARCHIVE keeps the REGISTER_APP-generated _register_Mixer_app() symbol
# alive through linking — main calls it via auto-generated AppRegistry_InitAll.
set_property(TARGET ${COMPONENT_LIB} PROPERTY WHOLE_ARCHIVE ON)
