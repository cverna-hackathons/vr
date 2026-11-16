cmake_minimum_required(VERSION 3.16)
project(vr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

add_executable(vr src/main.cpp src/text.cpp src/stereo.cpp src/cube.cpp src/video.cpp)
target_include_directories(vr SYSTEM PRIVATE third_party/glm-1.0.1)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavformat libavcodec libavutil libswscale libswresample)
target_link_libraries(vr PRIVATE PkgConfig::SDL2 PkgConfig::FFMPEG OpenGL::GL)
target_compile_options(vr PRIVATE -Wall -Wextra)


# --- Positional tracking camera (PTC) tools ---
add_library(ptc_camera STATIC src/ptc_camera.cpp)
add_executable(ptc_dump tools/ptc_dump.cpp)


# --- Direct DK2 headset driver (IMU + LED control) ---
pkg_check_modules(HIDAPI REQUIRED IMPORTED_TARGET hidapi-libusb)
add_library(dk2_hid STATIC src/dk2_hid.cpp)
target_include_directories(dk2_hid SYSTEM PUBLIC third_party/glm-1.0.1)
target_link_libraries(dk2_hid PUBLIC PkgConfig::HIDAPI)
add_executable(dk2_imu tools/dk2_imu.cpp)
target_link_libraries(dk2_imu PRIVATE dk2_hid)
target_link_libraries(ptc_dump PRIVATE ptc_camera dk2_hid)

add_library(ptc_tracker STATIC src/ptc_tracker.cpp)
target_link_libraries(ptc_tracker PUBLIC ptc_camera)
target_include_directories(ptc_tracker SYSTEM PUBLIC third_party/glm-1.0.1)
add_executable(ptc_ids tools/ptc_ids.cpp)
target_link_libraries(ptc_ids PRIVATE ptc_tracker dk2_hid)
add_executable(ptc_intrinsics tools/ptc_intrinsics.cpp)
target_link_libraries(ptc_intrinsics PRIVATE ptc_camera)

add_library(ptc_pose STATIC src/ptc_pose.cpp)
target_link_libraries(ptc_pose PUBLIC ptc_tracker)
add_executable(ptc_pose_test tools/ptc_pose.cpp)
target_link_libraries(ptc_pose_test PRIVATE ptc_pose dk2_hid)

add_library(head_tracker STATIC src/head_tracker.cpp)
target_link_libraries(head_tracker PUBLIC dk2_hid ptc_pose Threads::Threads)
target_link_libraries(vr PRIVATE head_tracker)
add_executable(track_debug tools/track_debug.cpp)
target_link_libraries(track_debug PRIVATE ptc_pose dk2_hid)
target_include_directories(track_debug SYSTEM PRIVATE third_party/glm-1.0.1)
add_executable(axis_check tools/axis_check.cpp)
target_link_libraries(axis_check PRIVATE head_tracker)

# --- Theater: separate executable, big screen + movie with sound ---
add_executable(theater src/theater.cpp src/movie.cpp src/stereo.cpp)
target_include_directories(theater SYSTEM PRIVATE third_party/glm-1.0.1)
target_link_libraries(theater PRIVATE head_tracker PkgConfig::SDL2 PkgConfig::FFMPEG OpenGL::GL Threads::Threads)
target_compile_options(theater PRIVATE -Wall -Wextra)

# --- Virtual display: mirror the built-in screen onto a monitor in front of the viewer ---
pkg_check_modules(XLIBS REQUIRED IMPORTED_TARGET x11 xext xfixes xrandr)
add_executable(display src/display.cpp src/xcapture.cpp src/stereo.cpp src/testpattern.cpp)
target_include_directories(display SYSTEM PRIVATE third_party/glm-1.0.1)
target_link_libraries(display PRIVATE head_tracker PkgConfig::SDL2 PkgConfig::XLIBS OpenGL::GL Threads::Threads)
target_compile_options(display PRIVATE -Wall -Wextra)

# --- Explore: walk through a text map in 3D ---
add_executable(explore src/explore.cpp src/mapdata.cpp src/stereo.cpp src/spatial_audio.cpp)
target_include_directories(explore SYSTEM PRIVATE third_party/glm-1.0.1)
target_link_libraries(explore PRIVATE head_tracker PkgConfig::SDL2 OpenGL::GL Threads::Threads)
target_compile_options(explore PRIVATE -Wall -Wextra)

# --- Sound preview: renders the explore sounds to WAV files ---
add_executable(sound_demo tools/sound_demo.cpp src/spatial_audio.cpp)
target_include_directories(sound_demo SYSTEM PRIVATE third_party/glm-1.0.1)
target_include_directories(sound_demo PRIVATE src)
target_link_libraries(sound_demo PRIVATE PkgConfig::SDL2 Threads::Threads)

# --- Guided calibration of the tracked position directions (accelerometer as ground truth) ---
add_executable(lean_calibrate tools/lean_calibrate.cpp)
target_link_libraries(lean_calibrate PRIVATE head_tracker)
