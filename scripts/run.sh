#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"
/Applications/CLion.app/Contents/bin/cmake/mac/aarch64/bin/cmake --build "$PROJECT_ROOT/cmake-build-release" --target Swarm -j 8

cd "$PROJECT_ROOT/cmake-build-release"
./Swarm

cd "$PROJECT_ROOT"
source .venv/bin/activate

cd "$PROJECT_ROOT/scripts/draw"
sleep 2
echo -e "0\n0\n3\n5\n14\nq" | python main.py