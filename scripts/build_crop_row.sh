#!/usr/bin/env bash
# Build our SDK + existing messages + observer from source. Never starts nodes.
set -euo pipefail
with_rknn=OFF
rknn_root=""
build_only=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-rknn) with_rknn=ON; shift ;;
    --rknn-root)
      [[ $# -ge 2 && -d "$2" ]] || { echo "--rknn-root needs an existing SDK directory" >&2; exit 2; }
      rknn_root=$(cd -- "$2" && pwd); shift 2 ;;
    --build-only) build_only=1; shift ;;
    *) echo "Usage: source /opt/ros/humble/setup.bash; bash scripts/build_crop_row.sh [--with-rknn] [--rknn-root PATH] [--build-only]" >&2; exit 2 ;;
  esac
done
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v colcon >/dev/null
command -v cmake >/dev/null
[[ "${ROS_DISTRO:-}" == humble ]] || { echo "Source ROS 2 Humble first" >&2; exit 2; }
# New prefixes avoid the old external RowNav_DIR CMake cache; no old files deleted.
build_dir="$repo_dir/build/crop-row-source"
install_dir="$repo_dir/install/crop-row-source"
cmake_args=(-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  -DRK_NAV_BUILD_TESTS=ON "-DRK_NAV_WITH_RKNN=$with_rknn" -DCMAKE_INSTALL_LIBDIR=lib)
if [[ -n "$rknn_root" ]]; then cmake_args+=("-DRKNN_ROOT=$rknn_root"); fi
colcon --log-base "$repo_dir/log/crop-row-source" build \
  --base-paths "$repo_dir/row_nav_sdk" "$repo_dir/weeder_msgs" "$repo_dir/crop_row_perception" \
  --build-base "$build_dir" --install-base "$install_dir" \
  --packages-up-to crop_row_perception --cmake-args "${cmake_args[@]}"
if [[ "$build_only" == 0 ]]; then
  colcon --log-base "$repo_dir/log/crop-row-source-test" test \
    --build-base "$build_dir" --install-base "$install_dir" \
    --packages-select row_nav_sdk crop_row_perception --return-code-on-test-failure
  colcon test-result --test-result-base "$build_dir" --verbose
fi
echo "Build complete. To use this overlay: source $install_dir/setup.bash"
