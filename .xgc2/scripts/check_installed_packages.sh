#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"

packages=()
case "${PACKAGE_GROUP}" in
  all)
    packages+=(ros-noetic-xgc2-onboard-detector-lv)
    packages+=(ros-noetic-xgc2-fapp-obj-state-msgs ros-noetic-xgc2-fapp-mot-mapping)
    ;;
  onboard-detector-lv)
    packages+=(ros-noetic-xgc2-onboard-detector-lv)
    ;;
  fapp|fapp-obj-state-msgs|fapp-mot-mapping)
    packages+=(ros-noetic-xgc2-fapp-obj-state-msgs ros-noetic-xgc2-fapp-mot-mapping)
    ;;
  *)
    echo "unknown package group: ${PACKAGE_GROUP}" >&2
    exit 1
    ;;
esac

for package in "${packages[@]}"; do
  dpkg-query -W -f='${Status}\n' "${package}" | grep -q "install ok installed"
done

source "${PREFIX}/setup.bash"

if [[ "${PACKAGE_GROUP}" == "all" || "${PACKAGE_GROUP}" == "onboard-detector-lv" ]]; then
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
fi

if [[ "${PACKAGE_GROUP}" == "all" || "${PACKAGE_GROUP}" == fapp* ]]; then
  test "$(rospack find fapp_obj_state_msgs)" = "${PREFIX}/share/fapp_obj_state_msgs"
  test "$(rospack find fapp_mot_mapping)" = "${PREFIX}/share/fapp_mot_mapping"
  test -f "${PREFIX}/include/fapp_obj_state_msgs/ObjectsStates.h"
  test -f "${PREFIX}/include/fapp_obj_state_msgs/State.h"
  test -x "${PREFIX}/lib/fapp_mot_mapping/fapp_mapping_node"
  test -f "${PREFIX}/share/fapp_mot_mapping/launch/fapp_mot_mapping.launch"

  ldd "${PREFIX}/lib/fapp_mot_mapping/fapp_mapping_node" >/dev/null
fi

echo "Installed package check passed"
