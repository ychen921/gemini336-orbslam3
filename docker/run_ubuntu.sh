#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-gemini336-orbslam3:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-gemini336-orbslam3-dev}"

PROJECT_ROOT="${PROJECT_ROOT:-$(
  cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)}"

CONTAINER_WORKSPACE="/workspaces/gemini336-orbslam3"

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
HOST_USER="$(id -un)"
CONTAINER_HOME="${HOME}/.docker-homes/${CONTAINER_NAME}"

mkdir -p "${CONTAINER_HOME}"

# Pangolin uses X11 for its viewer.
# If the container cannot connect to the display, run once on the host:
#
#   xhost +SI:localuser:"${HOST_USER}"
#
# or, if your Docker setup requires it:
#
#   xhost +local:docker

docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --user "${HOST_UID}:${HOST_GID}" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  --network host \
  --ipc host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e USER="${HOST_USER}" \
  -e LOGNAME="${HOST_USER}" \
  -e HOME="/home/${HOST_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="${CONTAINER_WORKSPACE}/config/fastdds_udp_only.xml" \
  -e FASTDDS_DEFAULT_PROFILES_FILE="${CONTAINER_WORKSPACE}/config/fastdds_udp_only.xml" \
  -e Pangolin_DIR="/workspaces/gemini336-orbslam3/external/install/pangolin/lib/cmake/Pangolin" \
  -e LD_LIBRARY_PATH="${CONTAINER_WORKSPACE}/external/install/pangolin/lib:${LD_LIBRARY_PATH:-}" \
  -v "${PROJECT_ROOT}:${CONTAINER_WORKSPACE}" \
  -v "${CONTAINER_HOME}:/home/${HOST_USER}" \
  -w "${CONTAINER_WORKSPACE}" \
  "${IMAGE_NAME}" \
  bash