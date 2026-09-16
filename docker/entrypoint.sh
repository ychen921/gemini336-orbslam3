#!/usr/bin/env bash
set -euo pipefail

# Docker exec inherits the environment supplied by the launcher; startup validates it once.
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_fastrtps_cpp" ]]; then
  profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
  if [[ -z "${profile}" || ! -f "${profile}" || ! -r "${profile}" || ! -s "${profile}" ]]; then
    echo "Fast DDS profile must be a nonempty readable file: ${profile:-<unset>}" >&2
    exit 1
  fi
  if [[ "${FASTDDS_DEFAULT_PROFILES_FILE:-${profile}}" != "${profile}" ]]; then
    echo "Fast DDS profile environment variables must reference the same file" >&2
    exit 1
  fi
  # Catch malformed XML before Fast DDS can fall back to its default transport settings.
  python3 - "${profile}" <<'PY'
import sys
import xml.etree.ElementTree as ET
try:
    ET.parse(sys.argv[1])
except (OSError, ET.ParseError) as error:
    sys.exit(f"Invalid Fast DDS profile: {error}")
PY
  echo "Fast DDS profile: ${profile}"
fi

# Preserve the ROS base image's environment setup and signal forwarding.
exec /ros_entrypoint.sh "$@"
