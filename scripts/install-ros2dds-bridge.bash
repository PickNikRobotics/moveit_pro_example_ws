#!/usr/bin/env bash
set -euo pipefail

ROS2DDS_VERSION="1.5.1"
CACHE_ROOT="${SO101_ROS2DDS_CACHE_DIR:-${XDG_CACHE_HOME:-${HOME}/.cache}/so101-ros2dds}"

case "$(uname -m)" in
  aarch64 | arm64)
    asset_arch="aarch64"
    asset_sha256="9bced19b07ea902c72ae403f2ab7221bb12f6378fb9fce4de4c9018f0cb78cdb"
    ;;
  x86_64 | amd64)
    asset_arch="x86_64"
    asset_sha256="abb86124d0650e8aaa39959d066b54931893bd34db326c287362fc88b2697dd8"
    ;;
  *)
    echo "error: unsupported ROS2DDS bridge architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

asset="zenoh-plugin-ros2dds-${ROS2DDS_VERSION}-${asset_arch}-unknown-linux-gnu-standalone.zip"
cache_dir="${CACHE_ROOT}/${ROS2DDS_VERSION}/${asset_arch}"
bridge="${cache_dir}/zenoh-bridge-ros2dds"

if [[ -x "$bridge" ]] &&
  "$bridge" --version 2>&1 | grep -q "zenoh-bridge-ros2dds v${ROS2DDS_VERSION}"; then
  printf '%s\n' "$bridge"
  exit 0
fi

download_dir="$(mktemp -d "${TMPDIR:-/tmp}/so101-ros2dds.XXXXXX")"
cleanup() {
  rm -rf -- "$download_dir"
}
trap cleanup EXIT INT TERM

curl --fail --location \
  --output "${download_dir}/${asset}" \
  "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ROS2DDS_VERSION}/${asset}"

if command -v sha256sum >/dev/null; then
  echo "${asset_sha256}  ${download_dir}/${asset}" |
    sha256sum --check --strict >&2
else
  actual_sha256="$(shasum -a 256 "${download_dir}/${asset}" | awk '{print $1}')"
  if [[ "$actual_sha256" != "$asset_sha256" ]]; then
    echo "error: ROS2DDS bridge checksum mismatch" >&2
    exit 1
  fi
fi

python3 -m zipfile -e "${download_dir}/${asset}" "$download_dir"
mkdir -p "$cache_dir"
install -m 0755 "${download_dir}/zenoh-bridge-ros2dds" "$bridge"
printf '%s\n' "$bridge"
