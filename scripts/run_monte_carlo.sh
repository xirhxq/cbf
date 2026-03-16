#!/bin/bash
# Run Monte Carlo experiments (20 runs)
# Each run uses randomized initial positions

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/cmake-build-release"
CONFIG_FILE="$PROJECT_ROOT/config/config_monte_carlo.json"
OUTPUT_BASE="$PROJECT_ROOT/data/monte_carlo"
LOG_DIR="$PROJECT_ROOT/logs"

# Create directories
mkdir -p "$OUTPUT_BASE"
mkdir -p "$LOG_DIR"

echo "Starting Monte Carlo experiments..."
echo "Output directory: $OUTPUT_BASE"
echo "Log directory: $LOG_DIR"
echo ""

for i in {1..20}; do
    RUN_DIR="$OUTPUT_BASE/run_$i"
    mkdir -p "$RUN_DIR"

    # Create temp config with output_path set
    TEMP_CONFIG="/tmp/mc_config_$i.json"
    cat "$CONFIG_FILE" | python3 -c "
import sys, json
data = json.load(sys.stdin)
data['output_path'] = '$RUN_DIR'
print(json.dumps(data, indent=2))
" > "$TEMP_CONFIG"

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting run $i..."
    LOG_FILE="$LOG_DIR/mc_run_$i.log"

    # Run simulation
    cd "$BUILD_DIR"
    ./Swarm "$TEMP_CONFIG" > "$LOG_FILE" 2>&1

    # Extract final search percentage from log
    FINAL_PCT=$(grep -oE "[0-9]+\.[0-9]+%" "$LOG_FILE" | tail -1)
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Run $i completed: ${FINAL_PCT} coverage"

    # Small delay between runs
    sleep 1
done

echo ""
echo "All runs completed!"
echo "Run 'python3 scripts/extract_monte_carlo.py' to extract results"
