#!/bin/bash
# Convenience wrapper - calls scripts/start_ros.sh
# Usage: ./start_ros.sh [--ekf]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/scripts/start_ros.sh" "$@"
