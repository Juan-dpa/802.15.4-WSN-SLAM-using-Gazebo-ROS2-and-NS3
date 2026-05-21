#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS="${1:-10}"
START_RUN="${2:-1}"
MAX_PARALLEL="${3:-$RUNS}"

cd "$NS3_DIR"

TRAJECTORY_FILE="scratch/Proyecto_ROS2_WSN/Inputs/trajectory-ulfer.csv"
OUTPUT_DIR="scratch/Proyecto_ROS2_WSN/Outputs"
LOG_DIR="$OUTPUT_DIR/logs"
NS_LOG_VALUE="SlamDataCollector=level_all|prefix_all"

mkdir -p "$OUTPUT_DIR" "$LOG_DIR"

# Compilamos primero una sola vez, evitando que las simulaciones paralelas
# intenten recompilar y se bloqueen entre sí.
./ns3 build

for ((i = 0; i < RUNS; i++)); do
    RNG=$((START_RUN + i))
    OUTPUT_FILE="$OUTPUT_DIR/slam_dataset_run${RNG}.csv"
    LOG_FILE="$LOG_DIR/slam_dataset_run${RNG}.log"

    (
        NS_GLOBAL_VALUE="RngRun=$RNG" \
        NS_LOG="$NS_LOG_VALUE" \
        ./ns3 run "topology --pcap=false --trajectoryFilename=$TRAJECTORY_FILE --outputFile=$OUTPUT_FILE" --no-build \
            > "$LOG_FILE" 2>&1
    ) &

    if (( (i + 1) % MAX_PARALLEL == 0 )); then
        wait
    fi
done

wait
