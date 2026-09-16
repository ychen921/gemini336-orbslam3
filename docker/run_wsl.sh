#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-gemini336-orbslam3:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-gemini336-orbslam3-wsl}"

PROJECT_ROOT="${PROJECT_ROOT:-$(
  cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)}"

CONTAINER_WORKSPACE="/workspaces/gemini336-orbslam3"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "${RMW_IMPLEMENTATION}" == "rmw_fastrtps_cpp" && ! -r "${PROJECT_ROOT}/configs/fastdds.xml" ]]; then
  echo "Missing readable Fast DDS profile: ${PROJECT_ROOT}/configs/fastdds.xml" >&2
  exit 1
fi

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
HOST_USER="$(id -un)"
HOST_HOSTNAME="$(hostname)"
CONTAINER_HOME="${CONTAINER_HOME:-${HOME}/.docker-homes/${CONTAINER_NAME}}"

mkdir -p "${CONTAINER_HOME}"

tty_args=()
if [[ -t 0 && -t 1 ]]; then tty_args=(-t); fi
if [[ $# -eq 0 ]]; then set -- bash; fi

docker run -i "${tty_args[@]}" --rm \
  --name "${CONTAINER_NAME}" \
  --hostname "${HOST_HOSTNAME}" \
  --user "${HOST_UID}:${HOST_GID}" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  --network host \
  --ipc host \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e USER="${HOST_USER}" \
  -e LOGNAME="${HOST_USER}" \
  -e HOME="/home/${HOST_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="${CONTAINER_WORKSPACE}/configs/fastdds.xml" \
  -e FASTDDS_DEFAULT_PROFILES_FILE="${CONTAINER_WORKSPACE}/configs/fastdds.xml" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -v "${PROJECT_ROOT}:${CONTAINER_WORKSPACE}" \
  -v "${CONTAINER_HOME}:/home/${HOST_USER}" \
  -w "${CONTAINER_WORKSPACE}" \
  --entrypoint "${CONTAINER_WORKSPACE}/docker/entrypoint.sh" \
  "${IMAGE_NAME}" \
  "$@"
