#!/usr/bin/env bash
set -euo pipefail

MODE="hardware"
INFERENCE="auto"
CHECKPOINT="${SO101_VLA_CHECKPOINT:-}"
ROBOT_HOST="${SO101_ROBOT_HOST:-so101-pi.tail337068.ts.net}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/so101-moveit-pro"

usage() {
  cat <<'EOF'
Usage: scripts/run-so101.bash [options] [-- moveit_pro run options]

Run the SO-101 workspace on the local Linux workstation.

Options:
  --hardware             Use the Pi-hosted ros2_control stack (default).
  --simulation           Use the local MuJoCo configuration.
  --with-inference       Start the local VLA inference sidecar.
  --without-inference    Do not start the inference sidecar.
  --checkpoint MODEL     Serve a local /models path or Hugging Face repo id.
  --robot-host HOST      Override the Pi hostname used for ROS and cameras.
  -h, --help             Show this help.

Inference is enabled automatically when a checkpoint is configured. HF_TOKEN
is inherited when set; otherwise the launcher securely reuses `hf auth token`.
EOF
}

extra_args=()
while (($#)); do
  case "$1" in
    --hardware)
      MODE="hardware"
      ;;
    --simulation)
      MODE="simulation"
      ;;
    --with-inference)
      INFERENCE="yes"
      ;;
    --without-inference)
      INFERENCE="no"
      ;;
    --checkpoint)
      shift
      (($#)) || {
        echo "error: --checkpoint requires a value" >&2
        exit 2
      }
      CHECKPOINT="$1"
      ;;
    --robot-host)
      shift
      (($#)) || {
        echo "error: --robot-host requires a value" >&2
        exit 2
      }
      ROBOT_HOST="$1"
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    --)
      shift
      extra_args+=("$@")
      break
      ;;
    *)
      extra_args+=("$1")
      ;;
  esac
  shift
done

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "error: run this launcher on the Linux workstation that hosts MoveIt Pro" >&2
  echo "The retired Apple container workflow is no longer supported." >&2
  exit 1
fi

command -v moveit_pro >/dev/null || {
  echo "error: moveit_pro is unavailable; install the MoveIt Pro Runtime first" >&2
  exit 1
}

mkdir -p "$STATE_DIR"

config_source="${WORKSPACE_DIR}/src/so101_moveit_config/config/vla_serving.yaml"
configured_checkpoint="$CHECKPOINT"
if [[ -z "$configured_checkpoint" ]]; then
  configured_checkpoint="$(python3 - "$config_source" <<'PY'
from pathlib import Path
import sys

for line in Path(sys.argv[1]).read_text().splitlines():
    if line.startswith("checkpoint:"):
        print(line.partition(":")[2].strip().strip("\"'"))
        break
PY
)"
fi

vla_config_dir="$(dirname "$config_source")"
if [[ -n "$CHECKPOINT" ]]; then
  vla_config_dir="${STATE_DIR}/vla-config"
  mkdir -p "$vla_config_dir"
  python3 - "$config_source" "${vla_config_dir}/vla_serving.yaml" "$CHECKPOINT" <<'PY'
import json
from pathlib import Path
import re
import sys

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
checkpoint = sys.argv[3]
text = source.read_text()
replacement = f"checkpoint: {json.dumps(checkpoint)}"
updated, count = re.subn(r"^checkpoint:.*$", replacement, text, count=1, flags=re.M)
if count != 1:
    raise SystemExit("vla_serving.yaml has no checkpoint field")
destination.write_text(updated)
PY
fi

if [[ "$INFERENCE" == "auto" ]]; then
  if [[ -n "$configured_checkpoint" ]]; then
    INFERENCE="yes"
  else
    INFERENCE="no"
  fi
fi

if [[ "$INFERENCE" == "yes" && -z "$configured_checkpoint" ]]; then
  echo "error: inference was requested but no SO-101 checkpoint is selected" >&2
  echo "Use --checkpoint <HF-repo-or-/models-path> or edit ${config_source}." >&2
  exit 1
fi

if [[ "$INFERENCE" == "yes" && -z "${HF_TOKEN:-}" ]] && command -v hf >/dev/null; then
  hf_token="$(hf auth token 2>/dev/null || true)"
  if [[ -n "$hf_token" ]]; then
    export HF_TOKEN="$hf_token"
  fi
  unset hf_token
fi

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID
export VLA_CONFIG_DIR="$vla_config_dir"

if command -v tailscale >/dev/null; then
  tailscale_ip="$(tailscale ip -4 2>/dev/null | head -n 1 || true)"
  if [[ -n "$tailscale_ip" ]]; then
    export MOVEIT_WEBRTC_ADDITIONAL_HOSTS="${MOVEIT_WEBRTC_ADDITIONAL_HOSTS:+${MOVEIT_WEBRTC_ADDITIONAL_HOSTS},}${tailscale_ip}"
  fi
fi

bridge_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$bridge_pid" ]]; then
    kill -TERM "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

config_package="so101_moveit_config"
if [[ "$MODE" == "hardware" ]]; then
  config_package="so101_moveit_hardware_config"
  export SO101_ROBOT_HOST="$ROBOT_HOST"
  export SO101_HEAD_RTSP_URL="${SO101_HEAD_RTSP_URL:-rtsp://${ROBOT_HOST}:8554/head}"
  export SO101_GRIPPER_RTSP_URL="${SO101_GRIPPER_RTSP_URL:-rtsp://${ROBOT_HOST}:8554/gripper}"
  export MOVEIT_WEBRTC_PASSTHROUGH="${MOVEIT_WEBRTC_PASSTHROUGH:-/so101/cameras/overhead/image_raw=${SO101_HEAD_RTSP_URL},/so101/cameras/wrist/image_raw=${SO101_GRIPPER_RTSP_URL}}"

  ros2dds_endpoint="${SO101_ROS2DDS_ENDPOINT:-tcp/${ROBOT_HOST}:7448}"
  if [[ ! "$ros2dds_endpoint" =~ ^(tcp|tls)/[A-Za-z0-9._:-]+:[0-9]+$ ]]; then
    echo "error: invalid SO101_ROS2DDS_ENDPOINT: ${ros2dds_endpoint}" >&2
    exit 1
  fi

  # The local DDS graph is intentionally loopback-only. Clear inherited host
  # overrides so the Runtime generates its matching default local profile;
  # Zenoh remains the only network transport between hosts.
  unset \
    USE_HOST_DDS \
    CYCLONEDDS_NETWORK_INTERFACE \
    CYCLONEDDS_PEER_ADDRESSES \
    CYCLONEDDS_USE_MULTICAST \
    CYCLONEDDS_MAX_AUTO_PARTICIPANT_INDEX \
    CYCLONEDDS_URI \
    FASTRTPS_DEFAULT_PROFILES_FILE

  bridge_binary="$("${SCRIPT_DIR}/install-ros2dds-bridge.bash")"
  bridge_log="${STATE_DIR}/ros2dds-bridge.log"
  ROS_DISTRO=jazzy \
  CYCLONEDDS_URI="${SCRIPT_DIR}/cyclonedds-local.xml" \
    "$bridge_binary" \
    --config "${SCRIPT_DIR}/ros2dds-moveit-control.json5" \
    client \
    --connect "$ros2dds_endpoint" \
    --no-multicast-scouting \
    --domain "$ROS_DOMAIN_ID" \
    >"$bridge_log" 2>&1 &
  bridge_pid=$!
  sleep 2
  if ! kill -0 "$bridge_pid" 2>/dev/null; then
    echo "error: ROS2DDS bridge exited during startup" >&2
    tail -50 "$bridge_log" >&2
    exit 1
  fi

  controller_is_active() {
    local response="$1"
    local name="$2"
    local flattened="${response//$'\n'/ }"
    local pattern="name:[[:space:]]*${name}[[:space:]]+state:[[:space:]]+id:[[:space:]]*[0-9]+[[:space:]]+label:[[:space:]]*active"
    [[ "$flattened" =~ $pattern ]]
  }

  # `moveit_pro shell` uses an ephemeral Runtime container when the stack is
  # not running. The transient-local activity message gives this preflight the
  # current typed controller contract without racing a WAN service proxy.
  controller_response="$(
    moveit_pro shell -s runtime -- bash -lc '
      source /opt/ros/jazzy/setup.bash
      source /opt/overlay_ws/install/setup.bash
      export ROS2CLI_NO_DAEMON=1
      timeout 15 ros2 topic echo \
        --once \
        --qos-durability transient_local \
        /controller_manager/activity \
        controller_manager_msgs/msg/ControllerManagerActivity
    ' 2>/dev/null || true
  )"

  if ! controller_is_active "$controller_response" "joint_state_broadcaster" ||
    ! controller_is_active "$controller_response" "joint_trajectory_controller" ||
    ! controller_is_active "$controller_response" "gripper_controller"; then
    echo "error: the Pi did not provide the typed ros2_control contract" >&2
    echo "Inspect ${STATE_DIR}/ros2dds-bridge.log and the Pi motion profile." >&2
    exit 1
  fi

  echo "SO-101 state, trajectory, and gripper controllers are reachable."
  if [[ "$controller_response" == *"joint_trajectory_admittance_controller"* ]]; then
    echo "The VLA trajectory controller is available. No motion has been commanded."
  elif [[ "$INFERENCE" == "yes" ]]; then
    echo "error: inference was requested, but the Pi has no joint_trajectory_admittance_controller" >&2
    echo "ExecutePolicy requires that controller for safe chunk stitching." >&2
    exit 1
  else
    echo "warning: joint_trajectory_admittance_controller is not installed on the Pi." >&2
    echo "Planning and teleoperation are available; Execute SO101 VLA Policy is not." >&2
  fi
else
  unset MOVEIT_WEBRTC_PASSTHROUGH
fi

run_args=(run --user-workspace "$WORKSPACE_DIR" -c "$config_package")
if [[ "$INFERENCE" == "yes" ]]; then
  run_args+=(--with-inference-server)
  echo "Starting ${config_package} with checkpoint ${configured_checkpoint}."
else
  echo "Starting ${config_package} without inference; no checkpoint is selected."
fi

moveit_pro "${run_args[@]}" "${extra_args[@]}"
