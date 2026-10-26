include(GNUInstallDirs)
add_library(rk_row_nav SHARED ${ROW_NAV_SDK_SOURCE_DIR}/src/sdk/rk_row_nav_c_api.cpp)
target_link_libraries(rk_row_nav PRIVATE rk_nav_core)
target_compile_definitions(rk_row_nav PRIVATE RK_ROW_NAV_BUILD=1)
target_include_directories(rk_row_nav PUBLIC
  $<BUILD_INTERFACE:${ROW_NAV_SDK_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(rk_row_nav PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1
  CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
if(APPLE)
  set_target_properties(rk_row_nav PROPERTIES INSTALL_RPATH "@loader_path")
  target_link_options(rk_row_nav PRIVATE "LINKER:-exported_symbols_list,${ROW_NAV_SDK_SOURCE_DIR}/cmake/sdk-symbols.apple")
elseif(UNIX)
  set_target_properties(rk_row_nav PROPERTIES INSTALL_RPATH "$ORIGIN")
  target_link_options(rk_row_nav PRIVATE "LINKER:--version-script=${ROW_NAV_SDK_SOURCE_DIR}/cmake/sdk-symbols.map")
endif()
install(TARGETS rk_row_nav EXPORT RowNavTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${ROW_NAV_SDK_SOURCE_DIR}/include/rk_row_nav.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT RowNavTargets NAMESPACE RowNav:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RowNav)
include(CMakePackageConfigHelpers)
configure_package_config_file(${ROW_NAV_SDK_SOURCE_DIR}/cmake/RowNavConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RowNavConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RowNav)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/RowNavConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RowNavConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/RowNavConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RowNav)
install(DIRECTORY ${ROW_NAV_SDK_SOURCE_DIR}/examples/sdk/ DESTINATION examples)
install(FILES ${ROW_NAV_SDK_SOURCE_DIR}/configs/bev.example.yaml
  ${ROW_NAV_SDK_SOURCE_DIR}/configs/camera-calibration.example.yaml DESTINATION config)
install(DIRECTORY ${ROW_NAV_SDK_SOURCE_DIR}/docs/ DESTINATION docs FILES_MATCHING PATTERN "*.md")
configure_file(${ROW_NAV_SDK_SOURCE_DIR}/cmake/sdk-build-info.txt.in
  ${CMAKE_CURRENT_BINARY_DIR}/sdk-build-info.txt @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sdk-build-info.txt DESTINATION .)
set(RK_ROW_NAV_RELEASE_MODEL "" CACHE FILEPATH "Optional local verified model to bundle")
if(RK_ROW_NAV_RELEASE_MODEL)
  if(NOT EXISTS "${RK_ROW_NAV_RELEASE_MODEL}")
    message(FATAL_ERROR "Release model does not exist")
  endif()
  install(FILES "${RK_ROW_NAV_RELEASE_MODEL}" DESTINATION models)
endif()
if(RK_NAV_BUILD_TESTS)
  enable_testing()
  add_executable(rk_sdk_c_header_test ${ROW_NAV_SDK_SOURCE_DIR}/tests/sdk/c_header_test.c)
  target_link_libraries(rk_sdk_c_header_test PRIVATE rk_row_nav)
  add_test(NAME sdk_c_header COMMAND rk_sdk_c_header_test)
  foreach(kind contract dynamic_contract)
    add_executable(rk_sdk_${kind}_test
      ${ROW_NAV_SDK_SOURCE_DIR}/tests/sdk/${kind}_test.cpp
      ${ROW_NAV_SDK_SOURCE_DIR}/tests/sdk/fake_segmenter.cpp
      ${ROW_NAV_SDK_SOURCE_DIR}/src/sdk/rk_row_nav_c_api.cpp)
    target_include_directories(rk_sdk_${kind}_test PRIVATE ${ROW_NAV_SDK_SOURCE_DIR}/tests)
    target_link_libraries(rk_sdk_${kind}_test PRIVATE rk_nav_core)
    add_test(NAME sdk_${kind} COMMAND rk_sdk_${kind}_test)
  endforeach()
endif()
