#!/usr/bin/env bash
# Build only the observer and its message dependency in a separate colcon prefix.
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: source /opt/ros/humble/setup.bash; bash scripts/build_crop_row.sh /absolute/sdk-prefix" >&2
  exit 2
fi
sdk_prefix=$(cd -- "$1" && pwd)
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v colcon >/dev/null
[[ "${ROS_DISTRO:-}" == humble ]] || { echo "Source ROS 2 Humble first" >&2; exit 2; }
python3 - "$repo_dir/dependencies/row_nav.lock.json" "$sdk_prefix/package-info.json" <<'PY'
import json
import sys
lock, package = (json.load(open(path, encoding='utf-8')) for path in sys.argv[1:])
if package['source_commit_at_packaging'] != lock['source_commit']:
    sys.exit('SDK source commit differs from dependency lock')
if package['tracked_changes_at_packaging'].strip():
    sys.exit('SDK was packaged with tracked source changes; publish a clean pinned package')
PY
(cd -- "$sdk_prefix" && sha256sum --quiet -c SHA256SUMS)
export CMAKE_PREFIX_PATH="$sdk_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon --log-base "$repo_dir/log/crop-row" build \
  --base-paths "$repo_dir/weeder_msgs" "$repo_dir/crop_row_perception" \
  --build-base "$repo_dir/build/crop-row" --install-base "$repo_dir/install/crop-row" \
  --packages-up-to crop_row_perception --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon --log-base "$repo_dir/log/crop-row-test" test \
  --build-base "$repo_dir/build/crop-row" --install-base "$repo_dir/install/crop-row" \
  --packages-select crop_row_perception --return-code-on-test-failure
colcon test-result --test-result-base "$repo_dir/build/crop-row" --verbose
