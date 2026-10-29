cmake_minimum_required(VERSION 3.16)
project(row_nav_calibration VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CTest)
option(ROW_NAV_CALIBRATION_BUILD_CAPTURE "Build camera capture (disable only for UI-only installations)" ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(ROW_NAV_CALIBRATION_BUILD_CAPTURE)
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)
  find_package(Threads REQUIRED)
  # Private header-only Frame/Status types from the SDK in this same repository.
  # No dependency on the old rk repository, no duplicate SDK implementation.
  set(sdk_headers "${CMAKE_CURRENT_SOURCE_DIR}/../row_nav_sdk/include")
  if(NOT EXISTS "${sdk_headers}/rk_nav/core/frame.hpp")
    message(FATAL_ERROR "Build from the laser_weed checkout containing row_nav_sdk")
  endif()
  add_executable(charuco_capture apps/charuco_capture.cpp src/video/camera_video_source.cpp)
  target_include_directories(charuco_capture PRIVATE include "${sdk_headers}" ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(charuco_capture PRIVATE ${OpenCV_LIBS} Threads::Threads)
  install(TARGETS charuco_capture RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(PROGRAMS scripts/row-nav-calibration DESTINATION bin)
install(DIRECTORY scripts/calibration DESTINATION share/${PROJECT_NAME}/scripts
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY tools output docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(FILES package.xml README.md requirements.txt DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME calibration_python COMMAND ${Python3_EXECUTABLE} -m pytest -q
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/python")
  set_tests_properties(calibration_python PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}" WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  find_program(NODE_EXECUTABLE node)
  if(NODE_EXECUTABLE)
    add_test(NAME calibration_ui_retry COMMAND ${NODE_EXECUTABLE} --test
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/js/test_bev_retry.cjs")
  endif()
endif()
