# xgc2-detection

ROS1 Noetic perception detection packages for XGC2.

This repository currently packages:

- `ros-noetic-xgc2-onboard-detector-lv`
- `ros-noetic-xgc2-fapp-obj-state-msgs`
- `ros-noetic-xgc2-fapp-mot-mapping`
- `ros-noetic-xgc2-detection`

`onboard_detector_lv` is imported from `LV-DOT/onboard_detector_lv`.
`fapp_obj_state_msgs` and `fapp_mot_mapping` are imported from FAPP as the point-cloud dynamic object state and mapping/tracking side of the stack.

## Build

```bash
.xgc2/scripts/check_package_compliance.sh
.xgc2/scripts/check_ros_packages.sh
.xgc2/scripts/build_debs_in_docker.sh --package-group onboard-detector-lv
.xgc2/scripts/build_debs_in_docker.sh --package-group fapp
```

The Docker build uses `ros:noetic-ros-base-focal`, builds the catkin package, installs into a staged root, packages the installed runtime payload, and can run the installed-package smoke check.

## APT Publish

The GitHub Actions workflow builds `amd64` and `arm64` package artifacts, builds meta packages, then publishes all debs with:

```bash
.xgc2/scripts/publish_apt_repo.sh --deb-dir debs
```

Publishing is skipped automatically when APT repository secrets are not configured.

## Runtime Notes

The C++ detector and fake detector are fully covered by APT dependencies. The YOLO Python nodes keep their model/config assets in the deb, but deep-learning Python packages are intentionally not hard-coded into Debian dependencies because `torch` and `ultralytics` are normally installed from pip or a CUDA-specific wheel index:

```bash
python3 -m pip install torch torchvision ultralytics
```
