# Integration contract

The runtime SDK remains in `row_nav_sdk`; calibration tools are maintained here.
The web workflow uses `charuco_capture`, the Python ChArUco solver, and the Python
BEV worker. It does not need a ROS node or a graphical desktop on the board.

The capture target privately consumes SDK Frame/Status headers from the same checkout.
The optional legacy C++ intrinsic calibration, validation and point-picker applications
are in `row_nav_sdk/apps`; enable `ROW_NAV_BUILD_LEGACY_CALIBRATION_TOOLS=ON` there.
Existing BEV CLI tools remain controlled by `ROW_NAV_BUILD_TOOLS`.

`row-nav-calibration` locates resources beside its installed prefix, never using cwd
or the old rk repository. Python dependencies are installed separately on the target
architecture; cross-compiling the C++ executable does not install Python dependencies.

Generated data defaults to `$XDG_DATA_HOME/laser_weed/calibration` (fallback
`~/.local/share/laser_weed/calibration`). Set `ROW_NAV_CALIBRATION_DATA` or the explicit
CLI directory options to override. Keep this directory outside build/install and Git.
Only the owner account should run the service. Binding 0.0.0.0 permits access on the
trusted direct Ethernet link; the tool has no Internet-facing authentication layer.

The runtime/current link is switched atomically after hardware-mode checks and human
review. Each version contains intrinsics.yaml, bev.yaml, manifest.json,
sdk-calibration.json and observer.calibration.yaml. The SDK initializes with the two
paths in sdk-calibration.json. The ROS overlay must be applied after the normal observer
configuration; it is a partial overlay, not a replacement for the full observer.yaml.
Restart/reinitialize perception after publishing. Versions contain absolute paths;
when moving to another computer, republish on that target rather than copying a stale
binding file. Build/install never overwrites generated calibration parameters.

Camera capture is an exclusive calibration operation: stop any other process that
owns the camera before entering hardware mode. This package never starts steering or
vehicle control. The migration itself does not validate a new camera or vehicle.

Source provenance: imported from rk commit 9734faf (UI retry fixes, guarded publication,
manual v4 and A2 print board). Existing rk files remain for history; future changes to
the delivered calibration workflow should be made in laser_weed/row_nav_calibration.
