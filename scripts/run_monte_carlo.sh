#!/bin/bash
# Run Monte Carlo experiments (20 runs)
# Each run uses randomized initial positions

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/cmake-build-release"
CONFIG_FILE="$PROJECT_ROOT/config/config_monte_carlo.json"

BATCH_TIMESTAMP=$(date '+%Y-%m-%d_%H-%M-%S')
OUTPUT_BASE="$PROJECT_ROOT/data/${BATCH_TIMESTAMP}_monte_carlo"

mkdir -p "$OUTPUT_BASE"

echo "Starting Monte Carlo experiments..."
echo "Output directory: $OUTPUT_BASE"
echo ""

for i in {1..20}; do
    RUN_SUFFIX="_run_$(printf '%02d' $i)"

    TEMP_CONFIG="/tmp/mc_config_$i.json"
    cat "$CONFIG_FILE" | python3 -c "
import sys, json
data = json.load(sys.stdin)
data['output_path'] = '$OUTPUT_BASE'
data['run_suffix'] = '$RUN_SUFFIX'
print(json.dumps(data, indent=2))
" > "$TEMP_CONFIG"

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting run $i..."

    cd "$BUILD_DIR"
    OUTPUT=$(./Swarm "$TEMP_CONFIG" 2>&1)
    echo "$OUTPUT"

    OUTPUT_DIR=$(echo "$OUTPUT" | grep -oE '\[OUTPUT_DIR\] .*' | sed 's/\[OUTPUT_DIR\] //')
    if [ -n "$OUTPUT_DIR" ]; then
        echo "$OUTPUT" > "$OUTPUT_DIR/output.log"
    fi

    FINAL_PCT=$(echo "$OUTPUT" | grep -oE "[0-9]+\.[0-9]+%" | tail -1)
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Run $i completed: ${FINAL_PCT} coverage"

    sleep 1
done

echo ""
echo "All runs completed!"
echo "Run 'python3 scripts/extract_monte_carlo.py' to extract results"
