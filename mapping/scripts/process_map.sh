#!/bin/bash
# process_map.sh - Process recorded sensor data into an occupancy grid map
#
# Usage:
#   ./process_map.sh <data_dir> [output_dir]
#
# Examples:
#   ./process_map.sh /tmp/exploration_data/exploration_20241215_143022
#   ./process_map.sh ~/exploration_data ~/maps/room1
#
# This script:
# 1. Validates the input data directory
# 2. Runs offline_mapper.py to build the map
# 3. Reports map statistics

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPER_SCRIPT="${SCRIPT_DIR}/../offline/offline_mapper.py"

usage() {
    echo "Usage: $0 <data_dir> [output_dir]"
    echo ""
    echo "Arguments:"
    echo "  data_dir    Input directory with recorded sensor data"
    echo "  output_dir  Output directory for map (default: ./map_output)"
    echo ""
    echo "Options:"
    echo "  --resolution, -r  Map resolution in meters (default: 0.05)"
    echo "  --size, -s        Map size in meters (default: 20)"
    echo "  --no-icp          Disable ICP pose refinement"
    echo "  --visualize, -v   Show visualization after processing"
    echo ""
    echo "Examples:"
    echo "  $0 /tmp/exploration_data/exploration_20241215_143022"
    echo "  $0 ~/data ~/maps/room1 --resolution 0.02"
    exit 1
}

log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

# Parse arguments
DATA_DIR=""
OUTPUT_DIR="./map_output"
RESOLUTION="0.05"
MAP_SIZE="20"
NO_ICP=""
VISUALIZE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--resolution)
            RESOLUTION="$2"
            shift 2
            ;;
        -s|--size)
            MAP_SIZE="$2"
            shift 2
            ;;
        --no-icp)
            NO_ICP="--no-icp"
            shift
            ;;
        -v|--visualize)
            VISUALIZE="--visualize"
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            if [[ -z "$DATA_DIR" ]]; then
                DATA_DIR="$1"
            elif [[ -z "$OUTPUT_DIR" ]] || [[ "$OUTPUT_DIR" == "./map_output" ]]; then
                OUTPUT_DIR="$1"
            fi
            shift
            ;;
    esac
done

# Validate input
if [[ -z "$DATA_DIR" ]]; then
    echo -e "${RED}Error: No data directory specified${NC}"
    usage
fi

if [[ ! -d "$DATA_DIR" ]]; then
    echo -e "${RED}Error: Data directory not found: $DATA_DIR${NC}"
    exit 1
fi

# Check for required files
if [[ ! -f "$DATA_DIR/metadata.json" ]]; then
    echo -e "${YELLOW}Warning: No metadata.json found in $DATA_DIR${NC}"
fi

SCANS_FILE=""
if [[ -f "$DATA_DIR/scans.jsonl.gz" ]]; then
    SCANS_FILE="$DATA_DIR/scans.jsonl.gz"
elif [[ -f "$DATA_DIR/scans.jsonl" ]]; then
    SCANS_FILE="$DATA_DIR/scans.jsonl"
fi

if [[ -z "$SCANS_FILE" ]]; then
    echo -e "${RED}Error: No scans file found in $DATA_DIR${NC}"
    echo "Expected: scans.jsonl or scans.jsonl.gz"
    exit 1
fi

ODOM_FILE=""
if [[ -f "$DATA_DIR/odom.jsonl.gz" ]]; then
    ODOM_FILE="$DATA_DIR/odom.jsonl.gz"
elif [[ -f "$DATA_DIR/odom.jsonl" ]]; then
    ODOM_FILE="$DATA_DIR/odom.jsonl"
fi

if [[ -z "$ODOM_FILE" ]]; then
    echo -e "${YELLOW}Warning: No odometry file found - map quality may be limited${NC}"
fi

# Check mapper script exists
if [[ ! -f "$MAPPER_SCRIPT" ]]; then
    echo -e "${RED}Error: offline_mapper.py not found at: $MAPPER_SCRIPT${NC}"
    exit 1
fi

# Display input info
log "${GREEN}Processing sensor data${NC}"
echo "  Input:      $DATA_DIR"
echo "  Output:     $OUTPUT_DIR"
echo "  Resolution: ${RESOLUTION}m"
echo "  Map size:   ${MAP_SIZE}m"
echo ""

# Count records in input files
if command -v zcat &> /dev/null && [[ "$SCANS_FILE" == *.gz ]]; then
    SCAN_COUNT=$(zcat "$SCANS_FILE" | wc -l)
else
    SCAN_COUNT=$(wc -l < "${SCANS_FILE%.gz}")
fi
log "Found $SCAN_COUNT scan records"

# Run mapper
log "Running offline mapper..."
python3 "$MAPPER_SCRIPT" \
    "$DATA_DIR" \
    --output "$OUTPUT_DIR" \
    --resolution "$RESOLUTION" \
    --size "$MAP_SIZE" \
    $NO_ICP \
    $VISUALIZE

# Check output
if [[ -f "$OUTPUT_DIR/map.png" ]]; then
    log "${GREEN}Map created successfully!${NC}"
    echo ""
    echo "Output files:"
    ls -la "$OUTPUT_DIR/"
    echo ""
    echo "To use with ROS2 map_server:"
    echo "  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$OUTPUT_DIR/map.yaml"
else
    log "${RED}Map creation failed${NC}"
    exit 1
fi
