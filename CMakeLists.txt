cmake_minimum_required(VERSION 3.12.4)

set(COMPONENT_ADD_INCLUDEDIRS
    src
    )
file(GLOB SRCS
     src/BBQ10Keyboard.cpp
     )
set(COMPONENT_SRCS ${SRCS})
# find_package(ARDUINO COMPONENTS arduino QUIET)
if(NOT ESP_PLATFORM)
  project(bbq10keyboard HOMEPAGE_URL https://github.com/solderparty/arduino_bbq10kbd)
else()
  set(BBQ10_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
  set(COMPONENT_REQUIRES driver esp32)

  register_component()
endif()

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG")