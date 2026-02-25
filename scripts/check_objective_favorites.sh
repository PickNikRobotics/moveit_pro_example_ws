#!/bin/bash
# Validates Objective favorite metadata:
# 1. Non-runnable Objectives must not be favorited
# 2. Each config package can have at most MAX_FAVORITES favorited Objectives

MAX_FAVORITES=8
errors=0

# Check no non-runnable Objectives are favorited
while IFS= read -r -d '' file; do
  if grep -q 'runnable="false"' "$file" && grep -q '_favorite="true"' "$file"; then
    echo "ERROR: Non-runnable Objective must not be favorited: ${file}"
    errors=$((errors + 1))
  fi
done < <(find src -name '*.xml' -path '*/objectives/*' -print0)

if [ "$errors" -gt 0 ]; then
  echo ""
  echo "Found $errors Objective(s) that are both non-runnable and favorited."
  echo "Objectives with <Metadata runnable=\"false\"/> must have _favorite=\"false\"."
fi

# Check favorite count does not exceed MAX_FAVORITES per config package
for pkg_dir in src/*/objectives/; do
  pkg_name=$(echo "$pkg_dir" | cut -d'/' -f2)
  count=$(grep -rl '_favorite="true"' --include='*.xml' "$pkg_dir" 2>/dev/null | wc -l | tr -d ' ')
  if [ "$count" -gt "$MAX_FAVORITES" ]; then
    echo "ERROR: ${pkg_name} has ${count} favorited Objectives (max ${MAX_FAVORITES})."
    echo "If everything is favorited, nothing is favorited."
    grep -rl '_favorite="true"' --include='*.xml' "$pkg_dir"
    errors=$((errors + 1))
  fi
done

if [ "$errors" -gt 0 ]; then
  exit 1
fi
