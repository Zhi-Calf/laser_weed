# Shared by the canonical SDK and the old rk development-tools build.
# Only this tree is the maintained source of runtime algorithms.
set(ROW_NAV_RUNTIME_SOURCES
  ${ROW_NAV_SDK_SOURCE_DIR}/src/calibration/camera_calibration.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/geometry/bev_calibration.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/geometry/bev_projector.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/geometry/dynamic_bev.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/geometry/centerline_fitter.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/inference/onnx_yolov8_segmenter.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/inference/yolov8_seg_postprocess.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/io/navigation_output.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/perception/row_extractor.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/perception/row_candidate_extractor.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/perception/multi_row_associator.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/perception/row_boundary_fitter.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/perception/row_temporal_tracker.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/pipeline/perception_pipeline.cpp
  ${ROW_NAV_SDK_SOURCE_DIR}/src/pipeline/tracked_perception_pipeline.cpp)
