#!/usr/bin/env zsh
set -eo pipefail

SCRIPT_DIR=${0:A:h}
FTG_ROOT=${FTG_ROOT:-$SCRIPT_DIR}
AUTODRIVE_ROOT=${AUTODRIVE_ROOT:-$HOME/Desktop/Racing/AutoDrive/AutoDRIVE-RoboRacer-Sim-Racing}
AUTODRIVE_VENV=${AUTODRIVE_VENV:-$AUTODRIVE_ROOT/autodrive_devkit/.venv_socketio_proto}
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.zsh}
SIM_IP=${SIM_IP:-127.0.0.1}
SIM_PORT=${SIM_PORT:-4567}
FTG_LOG_DIR=${FTG_LOG_DIR:-$FTG_ROOT/logs}

start_sim=false

for arg in "$@"; do
  if [[ "$arg" == "--with-sim" ]]; then
    start_sim=true
  elif [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
    echo "Usage: ./run_autodrive_ftg.zsh [--with-sim]"
    echo "  --with-sim   Also launch AutoDRIVE simulator binary"
    echo "Env overrides:"
    echo "  AUTODRIVE_ROOT, AUTODRIVE_VENV, FTG_ROOT, ROS_SETUP, SIM_IP, SIM_PORT"
    exit 0
  else
    echo "Unknown argument: $arg"
    echo "Use --help for usage"
    exit 1
  fi
done

require_file() {
  local p="$1"
  if [[ ! -f "$p" ]]; then
    echo "Missing file: $p"
    exit 1
  fi
}

require_dir() {
  local p="$1"
  if [[ ! -d "$p" ]]; then
    echo "Missing directory: $p"
    exit 1
  fi
}

require_file "$ROS_SETUP"
require_file "$AUTODRIVE_ROOT/install/setup.zsh"
require_file "$FTG_ROOT/install/setup.zsh"
require_file "$AUTODRIVE_VENV/bin/activate"
require_file "$AUTODRIVE_ROOT/autodrive_simulator/AutoDRIVE Simulator.x86_64"
require_dir "$AUTODRIVE_ROOT"
require_dir "$FTG_ROOT"
mkdir -p "$FTG_LOG_DIR"

cleanup_stale_bridge() {
  # Kill leftover bridge processes from prior runs to prevent port conflicts.
  local pids
  pids=$(ps -eo pid=,args= | awk '/autodrive_roboracer\/autodrive_bridge/ && !/awk/ {print $1}')
  if [[ -n "$pids" ]]; then
    echo "Stopping stale autodrive_bridge process(es): $pids"
    kill $pids 2>/dev/null || true
  fi
}

port_in_use() {
  # Return success if TCP LISTEN exists on SIM_PORT.
  ss -ltnp 2>/dev/null | grep -E ":${SIM_PORT}\\s" >/dev/null 2>&1
}

listener_details() {
  ss -ltnp 2>/dev/null | grep -E ":${SIM_PORT}\\s" || true
}

bridge_pid=""
sim_pid=""

cleanup() {
  set +e
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
  if [[ -n "$sim_pid" ]]; then
    kill "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [[ "$start_sim" == true ]]; then
  echo "Starting simulator on $SIM_IP:$SIM_PORT"
  "$AUTODRIVE_ROOT/autodrive_simulator/AutoDRIVE Simulator.x86_64" -ip "$SIM_IP" -port "$SIM_PORT" &
  sim_pid=$!
fi

cleanup_stale_bridge
if port_in_use; then
  echo "Port $SIM_PORT is already in use before bridge launch."
  listener_details
  echo "Stop that process or change SIM_PORT, then retry."
  exit 1
fi

echo "Starting AutoDRIVE bridge with protocol-compatible venv"
(
  unset COLCON_CURRENT_PREFIX AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
  source "$AUTODRIVE_VENV/bin/activate"
  source "$ROS_SETUP"
  source "$AUTODRIVE_ROOT/install/setup.zsh"
  export PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:${PYTHONPATH:-}"
  ros2 launch autodrive_roboracer bringup_headless.launch.py
) &
bridge_pid=$!

echo "Starting gap follower controller"
unset COLCON_CURRENT_PREFIX AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source "$ROS_SETUP"
source "$AUTODRIVE_ROOT/install/setup.zsh"
source "$FTG_ROOT/install/setup.zsh"
cd "$FTG_ROOT"
ros2 launch car_bringup car.launch.py controller:=gap_follow
