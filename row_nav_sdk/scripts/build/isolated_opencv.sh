#!/usr/bin/env bash
# Build only required CPU SDK modules. No sudo or system installation.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo 'usage: isolated_opencv.sh opencv-source new-work-root [jobs]' >&2; exit 1
fi
source_dir=$(cd "$1" && pwd)
root=$2
jobs=${3:-4}
[[ -f "$source_dir/CMakeLists.txt" ]] || exit 1
mkdir -p "$root"
root=$(cd "$root" && pwd)
cmake -S "$source_dir" -B "$root/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install" \
  -DBUILD_LIST=core,imgproc,imgcodecs,calib3d,dnn \
  -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF \
  -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF \
  -DWITH_CUDA=OFF -DWITH_OPENCL=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF \
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_GTK=OFF -DWITH_QT=OFF \
  -DOPENCV_DNN_OPENCL=OFF -DOPENCV_DNN_CUDA=OFF \
  -DCMAKE_INSTALL_RPATH="$root/install/lib"
cmake --build "$root/build" --parallel "$jobs"
cmake --install "$root/build"
printf 'OpenCV_DIR=%s/install/lib/cmake/opencv4\n' "$root"
