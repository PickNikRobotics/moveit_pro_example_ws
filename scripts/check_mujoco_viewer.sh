#!/bin/bash
# Ensures mujoco_viewer is not set to true in any config file.
# The MuJoCo viewer should always be disabled in committed config files.

errors=0

while IFS= read -r -d '' file; do
  if grep -qE 'mujoco_viewer:\s*(")?true(")?$' "$file"; then
    line=$(grep -nE 'mujoco_viewer:\s*(")?true(")?$' "$file")
    echo "ERROR: mujoco_viewer must be false: ${file}: ${line}"
    errors=$((errors + 1))
  fi
done < <(find src -name 'config.yaml' -print0)

if [ "$errors" -gt 0 ]; then
  echo ""
  echo "Found $errors config(s) with mujoco_viewer set to true."
  echo "The MuJoCo viewer should always be disabled in committed config files."
  exit 1
fi
