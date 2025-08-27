#!/bin/bash

set -e

PLOT=false
ANALYZE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --plot)
            PLOT=true
            shift
            ;;
        --analyze)
            ANALYZE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--plot] [--analyze]"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"
/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --build "$PROJECT_ROOT/cmake-build-release" --target Swarm -j 8

cd "$PROJECT_ROOT/cmake-build-release"
./Swarm

if [ "$ANALYZE" = true ]; then
    cd "$PROJECT_ROOT"
    source .venv/bin/activate

    LATEST_DATA=$(ls -t "$PROJECT_ROOT/data"/*/data.json | head -n1)
    if [ -n "$LATEST_DATA" ]; then
        echo "Analyzing data file: $LATEST_DATA"
        python "$PROJECT_ROOT/scripts/analysis/analyze_data.py" "$LATEST_DATA"
    else
        echo "No data files found for analysis"
    fi
fi

if [ "$PLOT" = true ]; then
    cd "$PROJECT_ROOT"
    source .venv/bin/activate
    
    cd "$PROJECT_ROOT/scripts/draw"
    sleep 2
    echo -e "0\n0\n3\n5\n14\nq" | python main.py
fi