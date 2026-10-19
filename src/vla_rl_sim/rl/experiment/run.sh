#!/usr/bin/env bash
# Run a vla_rl_sim prototype command in the experiment image (see Dockerfile), on an AMD ROCm GPU.
# Usage: run.sh "python3 \$RL/sft.py ..."; $RL is the prototype directory inside the container.
# Working directory, checkpoints and data: $MJBATCH_RL_DIR/exp19 (default ~/.cache/mjbatch_rl/exp19).
set -euo pipefail
SRC="$(cd "$(dirname "$0")/../../.." && pwd)"
CACHE="${MJBATCH_RL_DIR:-$HOME/.cache/mjbatch_rl}"
mkdir -p "$CACHE/exp19" "$HOME/.cache/huggingface"
# Numeric group ids: the container has no video/render groups of the host's ids.
exec docker run --rm --device /dev/kfd --device /dev/dri \
  --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)" --ipc=host \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e EXAMPLE_WS_SRC="$SRC" -e RUNTIME_MUJOCO_PATH="$CACHE/sidedeps36" \
  -e RL="$SRC/vla_rl_sim/rl" -e HF_HOME=/cache/hf \
  -v "$SRC:$SRC:ro" -v "$CACHE:$CACHE" -v "$HOME/.cache/huggingface:/cache/hf" \
  -w "$CACHE/exp19" --entrypoint bash vla-base-exp:latest -c "PYTHONPATH=$SRC/vla_rl_sim/rl $*"
