#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${ROS_PACKAGE_CHECK_WORK_DIR:-${REPO_ROOT}/.work/package-tests}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

rm -rf "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel" "${WORK_DIR}/install-root"
mkdir -p "${WORK_DIR}/src"
rsync -a --delete "${REPO_ROOT}/onboard_detector_lv/" "${WORK_DIR}/src/onboard_detector_lv/"

cd "${WORK_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "${WORK_DIR}/devel/setup.bash"
test "$(rospack find onboard_detector_lv)" = "${WORK_DIR}/src/onboard_detector_lv"
test -f "${WORK_DIR}/devel/include/onboard_detector_lv/GetDynamicObstacles.h"
test -x "${WORK_DIR}/devel/lib/onboard_detector_lv/detector_node_lv"
test -x "${WORK_DIR}/devel/lib/onboard_detector_lv/fake_detector_node_lv"

DESTDIR="${WORK_DIR}/install-root" catkin_make install \
  -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" \
  -DCATKIN_ENABLE_TESTING=OFF

test -d "${WORK_DIR}/install-root/opt/ros/${ROS_DISTRO}/share/onboard_detector_lv"
test -d "${WORK_DIR}/install-root/opt/ros/${ROS_DISTRO}/include/onboard_detector_lv"
test -x "${WORK_DIR}/install-root/opt/ros/${ROS_DISTRO}/lib/onboard_detector_lv/detector_node_lv"
test -x "${WORK_DIR}/install-root/opt/ros/${ROS_DISTRO}/lib/onboard_detector_lv/yolov11_detector_node.py"
test -f "${WORK_DIR}/install-root/opt/ros/${ROS_DISTRO}/lib/onboard_detector_lv/weights/yolo11n.pt"

echo "ROS package check passed"
