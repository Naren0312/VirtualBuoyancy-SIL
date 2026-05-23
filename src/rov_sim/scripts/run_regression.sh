#!/usr/bin/env bash
# ============================================================================
# run_regression.sh — Regression harness for ROV SIL simulator
#
# Usage:
#   ./run_regression.sh [scenario_file] [bag_output_dir]
#
# Arguments:
#   scenario_file   Path to the YAML scenario file (default: depth_hold.yaml)
#   bag_output_dir  Directory for rosbag output (default: vibu_sil/src/rov_sim/bags/)
#
# NOTE: Add bags/ to your .gitignore to avoid committing recorded data:
#   echo "bags/" >> vibu_sil/src/rov_sim/.gitignore
# ============================================================================
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SCENARIO_FILE="${1:-depth_hold.yaml}"
BAG_OUTPUT_DIR="${2:-${PACKAGE_DIR}/bags}"
SETTLE_TIME=2  # seconds to wait after scenario completes before stopping

# Timestamp-tagged bag directory
BAG_NAME="regression_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${BAG_OUTPUT_DIR}/${BAG_NAME}"

# Topics to record
TOPICS=(
    /setpoints
    /set_mode
    /sensor_data
    /thruster_command
)

# Path to the comparison script
COMPARE_SCRIPT="${SCRIPT_DIR}/compare_baseline.py"

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "[regression] Cleaning up…"
    # Kill child processes (launch, bag recorder)
    if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[regression] Stopping launch (PID ${LAUNCH_PID})…"
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        echo "[regression] Stopping bag recorder (PID ${BAG_PID})…"
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
    echo "[regression] Done."
}
trap cleanup EXIT

print_header() {
    echo "============================================================"
    echo "  ROV Regression Harness"
    echo "============================================================"
    echo "  Scenario file : ${SCENARIO_FILE}"
    echo "  Bag output    : ${BAG_DIR}"
    echo "  Topics        : ${TOPICS[*]}"
    echo "============================================================"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
print_header

# Create output directory
mkdir -p "${BAG_OUTPUT_DIR}"

# 1. Launch the simulator
echo "[regression] Launching rov_sim with scenario: ${SCENARIO_FILE}"
ros2 launch rov_sim rov_sim_launch.py \
    sim_backend:=mock \
    use_teleop:=false \
    scenario_file:="${SCENARIO_FILE}" &
LAUNCH_PID=$!
echo "[regression] Launch PID: ${LAUNCH_PID}"

# Give the launch a moment to bring up nodes
sleep 3

# 2. Start bag recording
echo "[regression] Starting rosbag recording → ${BAG_DIR}"
ros2 bag record \
    -o "${BAG_DIR}" \
    "${TOPICS[@]}" &
BAG_PID=$!
echo "[regression] Bag recorder PID: ${BAG_PID}"

# 3. Wait for scenario completion
echo "[regression] Waiting for /scenario_status == 'COMPLETE'…"
# ros2 topic echo --once will block until one message matching arrives.
# We filter for COMPLETE using grep in a loop to handle RUNNING messages.
while true; do
    MSG=$(ros2 topic echo --once /scenario_status std_msgs/msg/String 2>/dev/null || true)
    if echo "${MSG}" | grep -q "COMPLETE"; then
        echo "[regression] Scenario COMPLETE detected."
        break
    fi
    # If launch died, bail out
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[regression] ERROR: Launch process exited unexpectedly."
        exit 1
    fi
done

# 4. Settle period — let final messages flush into the bag
echo "[regression] Waiting ${SETTLE_TIME}s for data to settle…"
sleep "${SETTLE_TIME}"

# 5. Stop bag recording
echo "[regression] Stopping bag recorder…"
if kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
fi
BAG_PID=""  # prevent double-kill in cleanup

# 6. Stop the launch
echo "[regression] Stopping simulator launch…"
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
fi
LAUNCH_PID=""  # prevent double-kill in cleanup

# 7. Run comparison / baseline check
echo ""
echo "[regression] Running baseline comparison on: ${BAG_DIR}"
python3 "${COMPARE_SCRIPT}" "${BAG_DIR}"
RESULT=$?

if [[ ${RESULT} -eq 0 ]]; then
    echo ""
    echo "[regression] ✅ REGRESSION PASSED"
else
    echo ""
    echo "[regression] ❌ REGRESSION FAILED"
fi

exit ${RESULT}
