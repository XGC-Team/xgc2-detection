#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

required_files=(
  ".xgc2/product.yml"
  ".xgc2/scripts/build_debs_in_docker.sh"
  ".xgc2/scripts/check_installed_packages.sh"
  ".xgc2/scripts/check_package_compliance.sh"
  ".xgc2/scripts/check_ros_packages.sh"
  ".xgc2/scripts/package_debs.sh"
  ".xgc2/scripts/publish_apt_repo.sh"
  ".github/workflows/build-debs.yml"
  "README.md"
  "onboard_detector_lv/CMakeLists.txt"
  "onboard_detector_lv/package.xml"
  "onboard_detector_lv/srv/GetDynamicObstacles.srv"
  "onboard_detector_lv/launch/run_detector.launch"
  "onboard_detector_lv/cfg/detector_param.yaml"
  "onboard_detector_lv/scripts/yolo_detector/weights/yolo11n.pt"
)

for file in "${required_files[@]}"; do
  test -f "${REPO_ROOT}/${file}" || {
    echo "missing required file: ${file}" >&2
    exit 1
  }
done

grep -q "id: xgc2-detection" "${REPO_ROOT}/.xgc2/product.yml"
grep -q "ros-noetic-xgc2-onboard-detector-lv" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-detection" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "prune_installed_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "require_ros_package_payload" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "PACKAGE_VERSION:-1.0.0-1" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-vision-msgs" "${REPO_ROOT}/.xgc2/scripts/build_debs_in_docker.sh"
grep -q "publish_apt_repo.sh --deb-dir debs" "${REPO_ROOT}/.github/workflows/build-debs.yml"

echo "Package compliance check passed"
