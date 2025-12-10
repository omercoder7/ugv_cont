#!/bin/bash
# Convenience wrapper - calls scripts/rviz.sh
# Usage: ./rviz.sh <mode> [options]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/scripts/rviz.sh" "$@"
