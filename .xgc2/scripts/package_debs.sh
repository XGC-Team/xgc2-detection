#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.0.0-1}"
PACKAGE_GROUP="${PACKAGE_GROUP:-all}"
ARCH="$(dpkg --print-architecture)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    --arch)
      ARCH="$2"
      shift 2
      ;;
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "--output-dir is required" >&2
  exit 1
fi

if [[ "${PACKAGE_GROUP}" != "meta" && -z "${INSTALL_ROOT}" ]]; then
  echo "--install-root is required unless --package-group meta is used" >&2
  exit 1
fi

case "${ARCH}" in
  amd64|arm64)
    ;;
  *)
    echo "unsupported architecture: ${ARCH}" >&2
    exit 1
    ;;
esac

PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*_"${ARCH}".deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${description}
EOF
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/lib${ros_pkg}.so" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_pkg}" "${dst_root}"
}

require_ros_package_payload() {
  local ros_pkg="$1"
  local pkg_root="$2"

  if [[ ! -d "${pkg_root}${PREFIX}/share/${ros_pkg}" || ! -d "${pkg_root}${PREFIX}/lib/${ros_pkg}" ]]; then
    echo "missing installed payload for ROS package ${ros_pkg}; check catkin install() rules" >&2
    exit 1
  fi
}

prune_installed_package_payload() {
  local pkg_root="$1"
  local ros_pkg="$2"
  local share_dir="${pkg_root}${PREFIX}/share/${ros_pkg}"
  local lib_dir="${pkg_root}${PREFIX}/lib/${ros_pkg}"

  if [[ -d "${share_dir}" ]]; then
    rm -rf "${share_dir}/doc" "${share_dir}/docs"
    find "${share_dir}" -type f \( -iname '*.md' -o -iname '*.pdf' \) -delete
    find "${share_dir}" -depth -type d -empty -delete
  fi

  if [[ -d "${lib_dir}" ]]; then
    rm -rf "${lib_dir}/test_data" "${lib_dir}/__pycache__"
    find "${lib_dir}" -type f \( -iname 'result.png' -o -iname 'test.py' -o -iname '*.pyc' \) -delete
    find "${lib_dir}" -depth -type d -empty -delete
  fi

  find "${pkg_root}${PREFIX}/lib/python3/dist-packages" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true
}

build_ros_package_deb() {
  local package="$1"
  local ros_pkg="$2"
  local depends="$3"
  local description="$4"

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  require_ros_package_payload "${ros_pkg}" "${pkg_root}"
  prune_installed_package_payload "${pkg_root}" "${ros_pkg}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

onboard_detector_pkg="ros-noetic-xgc2-onboard-detector-lv"
meta_pkg="ros-noetic-xgc2-detection"

ros_base_depends="ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-geometry-msgs, ros-noetic-sensor-msgs, ros-noetic-nav-msgs"
detector_depends="${ros_base_depends}, ros-noetic-cv-bridge, ros-noetic-gazebo-msgs, ros-noetic-image-transport, ros-noetic-message-filters, ros-noetic-message-runtime, ros-noetic-pcl-conversions, ros-noetic-pcl-ros, ros-noetic-tf2-geometry-msgs, ros-noetic-vision-msgs, ros-noetic-visualization-msgs, libopencv-dev, libpcl-dev, python3"

build_onboard_detector_deb() {
  build_ros_package_deb \
    "${onboard_detector_pkg}" \
    "onboard_detector_lv" \
    "${detector_depends}" \
    "XGC2 LV-DOT onboard dynamic obstacle detector package"
}

build_meta_deb() {
  meta_root="${BUILD_DIR}/${meta_pkg}"
  rm -rf "${meta_root}"
  mkdir -p "${meta_root}"
  write_control \
    "${meta_root}" \
    "${meta_pkg}" \
    "${onboard_detector_pkg} (= ${VERSION})" \
    "XGC2 ROS1 perception detection package set"
  fakeroot dpkg-deb --build "${meta_root}" "${OUTPUT_DIR}/${meta_pkg}_${VERSION}_${ARCH}.deb" >/dev/null
}

case "${PACKAGE_GROUP}" in
  all)
    build_onboard_detector_deb
    build_meta_deb
    ;;
  onboard-detector-lv)
    build_onboard_detector_deb
    ;;
  meta)
    build_meta_deb
    ;;
  *)
    echo "unknown package group: ${PACKAGE_GROUP}" >&2
    exit 1
    ;;
esac

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
