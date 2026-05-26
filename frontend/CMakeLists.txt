cmake_minimum_required(VERSION 3.6)

project(EBIKEGUI)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(IMGUI_DIR  "../submodules/imgui")

         

add_library(${CMAKE_PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpuUsage.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/arc_progress_bar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguiGestures.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../timer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../valueTransition.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../cubicBezier.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../messagingUtils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../commonUtils.cpp
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_android.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
  ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

set(CMAKE_SHARED_LINKER_FLAGS
  "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate"
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
  IMGUI_IMPL_OPENGL_ES3
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  ../
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
  ${IMGUI_DIR}/misc/cpp
  ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
  android
  EGL
  GLESv3
  log
)
