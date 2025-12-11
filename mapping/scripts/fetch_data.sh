#!/bin/bash
# fetch_data.sh - Fetch exploration data from robot to local PC
#
# Usage:
#   ./fetch_data.sh [robot_host] [session_name]
#
# Examples:
#   ./fetch_data.sh                           # Fetch latest from default host
#   ./fetch_data.sh pi@192.168.1.100          # Fetch from specific robot
#   ./fetch_data.sh pi@robot exploration_20241215_143022  # Fetch specific session
#
# This script:
# 1. Lists available exploration sessions on the robot
# 2. Copies selected session to local machine
# 3. Validates the data

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default settings
DEFAULT_HOST="pi@ugv"
REMOTE_DATA_DIR="/tmp/exploration_data"
LOCAL_DATA_DIR="./exploration_data"

usage() {
    echo "Usage: $0 [robot_host] [session_name]"
    echo ""
    echo "Arguments:"
    echo "  robot_host    SSH host (default: $DEFAULT_HOST)"
    echo "  session_name  Specific session to fetch (default: latest)"
    echo ""
    echo "Options:"
    echo "  --list, -l    List available sessions without fetching"
    echo "  --output, -o  Local output directory (default: $LOCAL_DATA_DIR)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Fetch latest session"
    echo "  $0 pi@192.168.1.100                   # From specific host"
    echo "  $0 --list                             # List available sessions"
    echo "  $0 pi@robot exploration_20241215_*    # Fetch specific session"
    exit 1
}

log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

# Parse arguments
ROBOT_HOST="$DEFAULT_HOST"
SESSION_NAME=""
LIST_ONLY=false
OUTPUT_DIR="$LOCAL_DATA_DIR"

while [[ $# -gt 0 ]]; do
    case $1 in
        -l|--list)
            LIST_ONLY=true
            shift
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            if [[ "$1" == *@* ]]; then
                ROBOT_HOST="$1"
            elif [[ -z "$SESSION_NAME" ]]; then
                SESSION_NAME="$1"
            fi
            shift
            ;;
    esac
done

# Test SSH connection
log "Connecting to $ROBOT_HOST..."
if ! ssh -o ConnectTimeout=5 "$ROBOT_HOST" "echo 'OK'" &> /dev/null; then
    echo -e "${RED}Error: Cannot connect to $ROBOT_HOST${NC}"
    echo "Check that:"
    echo "  1. Robot is powered on and connected to network"
    echo "  2. SSH key is configured or password is ready"
    echo "  3. Host/IP is correct"
    exit 1
fi

# List available sessions
log "Listing exploration sessions on robot..."
SESSIONS=$(ssh "$ROBOT_HOST" "ls -1dt $REMOTE_DATA_DIR/exploration_* 2>/dev/null | head -10" || true)

if [[ -z "$SESSIONS" ]]; then
    echo -e "${YELLOW}No exploration sessions found in $REMOTE_DATA_DIR${NC}"
    echo "Run auto_scan.py with --log-sensors to create one"
    exit 1
fi

echo ""
echo "Available sessions:"
echo "-------------------"
ssh "$ROBOT_HOST" "ls -lht $REMOTE_DATA_DIR/ 2>/dev/null | grep exploration_ | head -10" || true
echo ""

if $LIST_ONLY; then
    exit 0
fi

# Select session to fetch
if [[ -z "$SESSION_NAME" ]]; then
    # Get latest session
    LATEST=$(ssh "$ROBOT_HOST" "ls -1dt $REMOTE_DATA_DIR/exploration_* 2>/dev/null | head -1")
    if [[ -z "$LATEST" ]]; then
        echo -e "${RED}Error: No sessions available${NC}"
        exit 1
    fi
    SESSION_PATH="$LATEST"
    SESSION_NAME=$(basename "$SESSION_PATH")
else
    # Use specified session
    if [[ "$SESSION_NAME" == exploration_* ]]; then
        SESSION_PATH="$REMOTE_DATA_DIR/$SESSION_NAME"
    else
        SESSION_PATH="$REMOTE_DATA_DIR/exploration_$SESSION_NAME"
    fi
fi

log "Fetching session: ${GREEN}$SESSION_NAME${NC}"

# Check session exists and get info
INFO=$(ssh "$ROBOT_HOST" "ls -la $SESSION_PATH/ 2>/dev/null" || true)
if [[ -z "$INFO" ]]; then
    echo -e "${RED}Error: Session not found: $SESSION_PATH${NC}"
    exit 1
fi

echo "Session contents:"
echo "$INFO"
echo ""

# Create local directory
mkdir -p "$OUTPUT_DIR"

# Copy data
log "Copying data to $OUTPUT_DIR/$SESSION_NAME..."
rsync -avz --progress "$ROBOT_HOST:$SESSION_PATH" "$OUTPUT_DIR/"

# Validate copied data
LOCAL_SESSION="$OUTPUT_DIR/$SESSION_NAME"
if [[ -f "$LOCAL_SESSION/metadata.json" ]]; then
    log "${GREEN}Data copied successfully!${NC}"
    echo ""
    echo "Session: $LOCAL_SESSION"
    cat "$LOCAL_SESSION/metadata.json" | python3 -m json.tool 2>/dev/null || cat "$LOCAL_SESSION/metadata.json"
    echo ""
    echo "To process this data:"
    echo "  ./mapping/scripts/process_map.sh $LOCAL_SESSION"
else
    log "${YELLOW}Warning: metadata.json not found - data may be incomplete${NC}"
fi
