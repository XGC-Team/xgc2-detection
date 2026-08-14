#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  --network host \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e PACKAGE_GROUP="${PACKAGE_GROUP}" \
  -v "${REPO_ROOT}:/workspace/detection:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libeigen3-dev \
      libopencv-dev \
      libpcl-dev \
      python3 \
      rsync \
      ros-noetic-cv-bridge \
      ros-noetic-gazebo-msgs \
      ros-noetic-geometry-msgs \
      ros-noetic-image-transport \
      ros-noetic-message-filters \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-nav-msgs \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-roscpp \
      ros-noetic-rospack \
      ros-noetic-rospy \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      ros-noetic-tf2-geometry-msgs \
      ros-noetic-vision-msgs \
      ros-noetic-visualization-msgs

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src
    case "${PACKAGE_GROUP}" in
      all)
        rsync -a --delete /workspace/detection/temp/onboard_detector_lv/ /workspace/work/src/onboard_detector_lv/
        rsync -a --delete /workspace/detection/temp/fapp_obj_state_msgs/ /workspace/work/src/fapp_obj_state_msgs/
        rsync -a --delete /workspace/detection/temp/fapp_mot_mapping/ /workspace/work/src/fapp_mot_mapping/
        ;;
      onboard-detector-lv)
        rsync -a --delete /workspace/detection/temp/onboard_detector_lv/ /workspace/work/src/onboard_detector_lv/
        ;;
      fapp|fapp-obj-state-msgs|fapp-mot-mapping)
        rsync -a --delete /workspace/detection/temp/fapp_obj_state_msgs/ /workspace/work/src/fapp_obj_state_msgs/
        rsync -a --delete /workspace/detection/temp/fapp_mot_mapping/ /workspace/work/src/fapp_mot_mapping/
        ;;
      *)
        echo "unknown package group: ${PACKAGE_GROUP}" >&2
        exit 1
        ;;
    esac

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/detection/.xgc2/scripts/package_debs.sh \
      --package-group "${PACKAGE_GROUP}" \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/*.deb
      /workspace/detection/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
