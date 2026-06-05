#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
packages=(
  ros-noetic-xgc2-onboard-detector-lv
)

for package in "${packages[@]}"; do
  dpkg-query -W -f='${Status}\n' "${package}" | grep -q "install ok installed"
done

source "${PREFIX}/setup.bash"
test "$(rospack find onboard_detector_lv)" = "${PREFIX}/share/onboard_detector_lv"
test -d "${PREFIX}/include/onboard_detector_lv"
test -f "${PREFIX}/include/onboard_detector_lv/GetDynamicObstacles.h"
test -f "${PREFIX}/include/onboard_detector_lv/dynamicDetector.h"
test -x "${PREFIX}/lib/onboard_detector_lv/detector_node_lv"
test -x "${PREFIX}/lib/onboard_detector_lv/fake_detector_node_lv"
test -x "${PREFIX}/lib/onboard_detector_lv/yolo_detector_node.py"
test -x "${PREFIX}/lib/onboard_detector_lv/yolov11_detector_node.py"
test -f "${PREFIX}/lib/onboard_detector_lv/weights/yolo11n.pt"
test -f "${PREFIX}/lib/onboard_detector_lv/weights/weight_AP05:0.253207_280-epoch.pth"
test -f "${PREFIX}/share/onboard_detector_lv/launch/run_detector.launch"
test -f "${PREFIX}/share/onboard_detector_lv/cfg/detector_param.yaml"

ldd "${PREFIX}/lib/onboard_detector_lv/detector_node_lv" >/dev/null
ldd "${PREFIX}/lib/onboard_detector_lv/fake_detector_node_lv" >/dev/null
ldd "${PREFIX}/lib/libonboard_detector_lv.so" >/dev/null

echo "Installed package check passed"
